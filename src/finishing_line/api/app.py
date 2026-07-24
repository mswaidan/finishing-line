"""FastAPI layer — the HMI's only interface to the line.

Thin by design: every route is a direct call into LineController, which owns
all thread-safety. The HMI (served at /) never talks to the robot or the
ClearCore; that rule is the architecture (CLAUDE.md), and this module is where
it is enforced by construction — there is simply nothing else to call.

No auth: this binds to the shop LAN for operators. Do not expose it wider.
"""

from __future__ import annotations

import asyncio
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, Field

from ..process.controller import LineController

_HMI = Path(__file__).with_name("hmi.html")


class BatchRequest(BaseModel):
    product: str = Field(pattern="^(cube|browser)$")
    part_ids: list[str] | None = None
    count: int | None = Field(default=None, ge=1, le=8)


class RunRequest(BaseModel):
    enabled: bool


class HaltRequest(BaseModel):
    reason: str = "operator halt"


class ProductRequest(BaseModel):
    product: str = Field(pattern="^(cube|browser)$")


class FaultAckRequest(BaseModel):
    #: station name (IF/S/FD) -> part id; omit to accept current belief
    occupancy: dict[str, str] | None = None
    beat: str | None = Field(default=None, pattern="^P[1-4]$")


def create_app(controller: LineController) -> FastAPI:
    app = FastAPI(title="finishing-line", docs_url="/docs")
    seq = {"n": 0}

    @app.get("/", response_class=HTMLResponse)
    def hmi() -> str:
        return _HMI.read_text(encoding="utf-8")

    @app.get("/state")
    def state() -> dict:
        return controller.snapshot()

    @app.post("/run")
    def run(req: RunRequest) -> dict:
        controller.set_running(req.enabled)
        return {"enabled": req.enabled}

    @app.post("/halt")
    def halt(req: HaltRequest) -> dict:
        controller.halt(req.reason)
        return {"halted": True, "reason": req.reason}

    @app.post("/batch")
    def batch(req: BatchRequest) -> dict:
        if req.part_ids is None and req.count is None:
            raise HTTPException(422, "provide part_ids or count")
        ids = req.part_ids
        if ids is None:
            ids = []
            for _ in range(req.count or 0):
                seq["n"] += 1
                ids.append(f"{req.product[0]}{seq['n']:03d}")
        try:
            staged = controller.declare_batch(req.product, ids)
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc
        return {"staged": staged}

    @app.post("/product")
    def product(req: ProductRequest) -> dict:
        """Continuous-intake product switch (legacy mode only)."""
        if not hasattr(controller, "set_product"):
            raise HTTPException(409, "this mode batches by declaration, not intake")
        controller.set_product(req.product)
        return {"intake_product": req.product}

    @app.post("/fault/ack")
    def fault_ack(req: FaultAckRequest) -> dict:
        resumed, reason = controller.ack_fault(req.occupancy, req.beat)
        if not resumed:
            raise HTTPException(409, reason or "resume rejected")
        return {"resumed": True}

    @app.websocket("/events")
    async def events(ws: WebSocket) -> None:
        """State stream, ~4 Hz. Snapshot-based, not delta-based: at this size
        (a few KB) deltas are complexity with no payoff, and a reconnecting
        HMI needs the full picture anyway.
        """
        await ws.accept()
        try:
            while True:
                await ws.send_json(controller.snapshot())
                await asyncio.sleep(0.25)
        except Exception:  # incl. WebSocketDisconnect
            # A refresh/tab-close can kill the socket mid-send, which raises
            # transport errors beyond WebSocketDisconnect — all of them just
            # mean "client gone"; stop streaming, never log-spam.
            pass

    return app
