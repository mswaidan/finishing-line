// Finishing line ClearCore firmware — the tick.
//
// C++ twin of sim/fake_clearcore.py::_tick, which is the executable spec:
// tests/test_fake_clearcore.py pins down every behaviour implemented here
// (echo handshake, zone move lifecycle with acks, EDGE-triggered sensor
// stops, shutter feedback, fail-ON watchdog that arms on first heartbeat).
// If this file and the fake disagree, one of them is wrong — fix the pair.
//
// Differences from the fake, all physical:
//   - distance moves finish on StepsComplete(), not a duration model
//   - sensors are debounced (DEBOUNCE_MS) — noise must not stop a belt
//   - shutter feedback comes from real end switches, not a timer

#include "firmware.h"

#include "ClearCore.h"
#include "Ethernet.h"
#include "io_map.h"
#include "modbus_tcp.h"
#include "registers.h"

namespace {

RegisterFile regs;
ModbusTcpServer server(regs);

// ------------------------------------------------------------- debouncing
struct Debounced {
  bool stable = false;
  bool candidate = false;
  uint32_t candidateSinceMs = 0;

  bool update(bool raw, uint32_t nowMs) {
    if (raw != candidate) {
      candidate = raw;
      candidateSinceMs = nowMs;
    } else if (raw != stable && (nowMs - candidateSinceMs) >= DEBOUNCE_MS) {
      stable = raw;
    }
    return stable;
  }
};

Debounced dbIf, dbS, dbFd, dbHandZ1, dbHandZ2, dbShutOpen, dbShutClosed;
Debounced dbInq, dbOut;

// ------------------------------------------------------------------ zones
struct ZoneBlock {
  // Constructor instead of aggregate init: the ClearCore toolchain is
  // gnu++11, where default member initializers make a struct non-aggregate.
  ZoneBlock(uint16_t mode, uint16_t dist, uint16_t reqid, uint16_t dir,
            uint16_t target, uint16_t state, uint16_t ack, MotorDriver *m)
      : modeReg(mode), distReg(dist), reqidReg(reqid), dirCoil(dir),
        targetReg(target), stateReg(state), ackReg(ack), motor(m),
        lastReqId(0), moveActive(false), edgeArmed(false), edgeSensorReg(0),
        edgeWantRising(true), edgePrev(false) {}

  uint16_t modeReg, distReg, reqidReg, dirCoil, targetReg;
  uint16_t stateReg, ackReg;
  MotorDriver *motor;

  uint16_t lastReqId;
  bool moveActive;       // a distance move is in flight
  bool edgeArmed;        // a sensor-stop move is in flight
  uint16_t edgeSensorReg;
  bool edgeWantRising;
  bool edgePrev;
};

ZoneBlock zone1 = {REG_Z1_MODE, REG_Z1_DIST, REG_Z1_REQID,
                   REG_Z1_DIR,   REG_Z1_TARGET,   REG_Z1_STATE,
                   REG_Z1_ACK,   &Z1_BELT};
ZoneBlock zone2 = {REG_Z2_MODE, REG_Z2_DIST, REG_Z2_REQID,
                   REG_Z2_DIR,   REG_Z2_TARGET,   REG_Z2_STATE,
                   REG_Z2_ACK,   &Z2_BELT};

// --------------------------------------------------------------- watchdog
uint16_t lastHeartbeat = 0;
uint32_t heartbeatSeenAtMs = 0;
bool watchdogArmed = false;

// ------------------------------------------------- serial diagnostics (USB)
// Bench bring-up only: the register contract (what the fake mirrors) is
// unchanged — this is firmware-side observability with no fake counterpart,
// like debounce and StepsComplete above.
bool linkWasUp = true;          // setup() blocks until link is up, so start true
uint32_t lastLinkCheckMs = 0;

void printNet(const char *tag) {
  IPAddress ip(CC_IP_ADDRESS);
  Serial.print(tag);
  Serial.print("ip=");
  Serial.print(ip[0]); Serial.print('.'); Serial.print(ip[1]); Serial.print('.');
  Serial.print(ip[2]); Serial.print('.'); Serial.println(ip[3]);
}

bool sensorValue(uint16_t reg) { return regs.discrete[reg]; }

int32_t velocityLimit() {
  return regs.holding[CMD_VELOCITY] ? regs.holding[CMD_VELOCITY] : DEFAULT_VELOCITY_SPS;
}
int32_t accelLimit() {
  return regs.holding[CMD_ACCELERATION] ? regs.holding[CMD_ACCELERATION]
                                        : DEFAULT_ACCEL_SPS2;
}

void tickZone(ZoneBlock &z, bool tripped) {
  z.motor->VelMax(velocityLimit());
  z.motor->AccelMax(accelLimit());

  if (tripped) {
    z.motor->MoveStopDecel();
    z.moveActive = false;
    z.edgeArmed = false;
    regs.inputRegs[z.stateReg] = STATE_READY;
    return;
  }

  uint16_t mode = regs.holding[z.modeReg];
  int dirSign = regs.coils[z.dirCoil] ? 1 : -1;

  switch (mode) {
    case MODE_CONTINUOUS:
      z.moveActive = false;
      z.edgeArmed = false;
      z.motor->MoveVelocity(dirSign * velocityLimit());
      regs.inputRegs[z.stateReg] = STATE_MOVING;
      break;

    case MODE_IDLE:
      z.moveActive = false;
      z.edgeArmed = false;
      z.motor->MoveStopDecel();
      regs.inputRegs[z.stateReg] = STATE_READY;
      break;

    case MODE_DISTANCE: {
      uint16_t reqid = regs.holding[z.reqidReg];
      if (reqid != z.lastReqId) {
        z.lastReqId = reqid;
        z.edgeArmed = false;
        z.motor->Move(dirSign * (int32_t)regs.holding[z.distReg]);
        z.moveActive = true;
        regs.inputRegs[z.stateReg] = STATE_MOVING;
        // Ack AFTER the move is issued: the ack promises "this id's move is
        // running", never merely "I saw the number".
        regs.inputRegs[z.ackReg] = reqid;
      } else if (z.moveActive && z.motor->StepsComplete()) {
        z.moveActive = false;
        regs.inputRegs[z.stateReg] = STATE_READY;
      }
      break;
    }

    case MODE_SENSOR_STOP: {
      uint16_t reqid = regs.holding[z.reqidReg];
      if (reqid != z.lastReqId) {
        z.lastReqId = reqid;
        z.moveActive = false;
        uint16_t raw = regs.holding[z.targetReg];
        z.edgeWantRising = !(raw & TARGET_FALLING_FLAG);
        switch (raw & ~TARGET_FALLING_FLAG) {
          case TARGET_F1_EYE: z.edgeSensorReg = REG_F1_EYE; break;
          case TARGET_O_EYE: z.edgeSensorReg = REG_O_EYE; break;
          case TARGET_F2_EYE: z.edgeSensorReg = REG_F2_EYE; break;
          case TARGET_Z1_EYE: z.edgeSensorReg = REG_Z1_EYE; break;
          case TARGET_Z2_EYE: z.edgeSensorReg = REG_Z2_EYE; break;
          default:
            // Unknown target: refuse to run. READY without ack tells the
            // orchestrator the move was not accepted (ack poll times out).
            regs.inputRegs[z.stateReg] = STATE_READY;
            return;
        }
        // EDGE semantics: record the level at arm time; only a TRANSITION to
        // the target polarity stops the belt. A level already at target must
        // not trip — the vacate-then-fill case this mode exists to get right.
        z.edgePrev = sensorValue(z.edgeSensorReg);
        z.edgeArmed = true;
        z.motor->MoveVelocity(dirSign * velocityLimit());
        regs.inputRegs[z.stateReg] = STATE_MOVING;
        regs.inputRegs[z.ackReg] = reqid;
      } else if (z.edgeArmed) {
        bool level = sensorValue(z.edgeSensorReg);
        if (level != z.edgePrev && level == z.edgeWantRising) {
          z.edgeArmed = false;
          z.motor->MoveStopDecel();
          regs.inputRegs[z.stateReg] = STATE_READY;
        }
        z.edgePrev = level;
      }
      break;
    }

    default:
      // Unknown mode: safest is a controlled stop.
      z.motor->MoveStopDecel();
      regs.inputRegs[z.stateReg] = STATE_READY;
      break;
  }
}

}  // namespace

void firmwareSetup() {
  // USB serial for bench diagnostics. Bounded wait so a headless boot (no host
  // attached) proceeds after 2 s instead of hanging on `while (!Serial)`.
  Serial.begin(115200);
  uint32_t serialWait = Milliseconds();
  while (!Serial && Milliseconds() - serialWait < 2000) continue;
  Serial.println();
  Serial.println("=== finishing-line ClearCore firmware (rewrite) ===");

  // Motors: open-loop step/dir, legacy clocking and polarity (ino:192-207).
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_LOW);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
  MotorDriver *motors[] = {&Z1_BELT, &Z2_BELT, &IN_BELT, &O_BRUSH};
  for (MotorDriver *m : motors) {
    m->VelMax(DEFAULT_VELOCITY_SPS);
    m->AccelMax(DEFAULT_ACCEL_SPS2);
    m->PolarityInvertSDEnable(INVERT_STEP_ENABLE);
    m->PolarityInvertSDDirection(INVERT_STEP_DIRECTION);
    m->EnableRequest(true);
  }
  O_BRUSH.VelMax(BRUSH_VELOCITY_SPS);
  O_BRUSH.AccelMax(BRUSH_ACCEL_SPS2);

  PIN_F1_EYE.Mode(Connector::INPUT_DIGITAL);
  PIN_O_EYE.Mode(Connector::INPUT_DIGITAL);
  PIN_F2_EYE.Mode(Connector::INPUT_DIGITAL);
  PIN_Z2_EYE.Mode(Connector::INPUT_DIGITAL);
  PIN_Z1_EYE.Mode(Connector::INPUT_DIGITAL);
  PIN_IN_EYE.Mode(Connector::INPUT_DIGITAL);
  PIN_OUT_EYE.Mode(Connector::INPUT_DIGITAL);
  PIN_SH_OPEN_EYE.Mode(Connector::INPUT_DIGITAL);
  PIN_SH_CLOSED_EYE.Mode(Connector::INPUT_DIGITAL);
  PIN_F1_FAN.Mode(Connector::OUTPUT_DIGITAL);
  PIN_F2_FAN.Mode(Connector::OUTPUT_DIGITAL);
  PIN_SH_OPEN_SOL.Mode(Connector::OUTPUT_DIGITAL);
  PIN_SH_CLOSE_SOL.Mode(Connector::OUTPUT_DIGITAL);

  // Static IP — see io_map.h for why DHCP is deliberately gone.
  IPAddress ip(CC_IP_ADDRESS);
  IPAddress gw(CC_GATEWAY);
  IPAddress mask(CC_NETMASK);
  uint8_t mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE};  // legacy MAC (ino:27)
  Ethernet.begin(mac, ip, gw, gw, mask);
  printNet("net: ");
  Serial.print("waiting for Ethernet link");
  while (Ethernet.linkStatus() == LinkOFF) {
    Serial.print('.');
    delay(500);  // no cable, nothing to do — same posture as legacy
  }
  Serial.println(" UP");
  server.begin();
  Serial.println("modbus: listening on :502");

  regs.inputRegs[REG_SERVER_STATE] = STATE_READY;  // legacy observability
  regs.inputRegs[REG_Z1_STATE] = STATE_READY;
  regs.inputRegs[REG_Z2_STATE] = STATE_READY;
  regs.holding[REG_Z1_MODE] = MODE_IDLE;
  regs.holding[REG_Z2_MODE] = MODE_IDLE;
}

void firmwareLoop() {
  uint32_t nowMs = Milliseconds();
  server.poll();

  // ---- serial: report Ethernet link transitions (bench cable work). Polled
  // at 4 Hz so we never hammer the PHY from the fast control loop.
  if (nowMs - lastLinkCheckMs >= 250) {
    lastLinkCheckMs = nowMs;
    bool linkUp = Ethernet.linkStatus() != LinkOFF;
    if (linkUp != linkWasUp) {
      linkWasUp = linkUp;
      if (linkUp) printNet("link UP  ");
      else Serial.println("link DOWN");
    }
  }

  // ---- sensors -> discrete inputs (debounced; these gate belt stops)
  regs.discrete[REG_F1_EYE] = dbIf.update(PIN_F1_EYE.State(), nowMs);
  regs.discrete[REG_O_EYE] = dbS.update(PIN_O_EYE.State(), nowMs);
  regs.discrete[REG_F2_EYE] = dbFd.update(PIN_F2_EYE.State(), nowMs);
  regs.discrete[REG_Z1_EYE] = dbHandZ1.update(PIN_Z1_EYE.State(), nowMs);
  regs.discrete[REG_Z2_EYE] = dbHandZ2.update(PIN_Z2_EYE.State(), nowMs);
  regs.discrete[REG_IN_EYE] = dbInq.update(PIN_IN_EYE.State(), nowMs);
  regs.discrete[REG_OUT_EYE] = dbOut.update(PIN_OUT_EYE.State(), nowMs);
  bool shutOpen = dbShutOpen.update(PIN_SH_OPEN_EYE.State(), nowMs);
  bool shutClosed = dbShutClosed.update(PIN_SH_CLOSED_EYE.State(), nowMs);

  // ---- legacy echo handshake (updateLocals of the old firmware)
  regs.inputRegs[ECHO_MOTION_MODE] = regs.holding[CMD_MOTION_MODE];
  regs.discrete[ECHO_DIRECTION] = regs.coils[CMD_DIRECTION];
  regs.inputRegs[ECHO_VELOCITY] = regs.holding[CMD_VELOCITY];
  regs.inputRegs[ECHO_ACCELERATION] = regs.holding[CMD_ACCELERATION];
  regs.inputRegs[ECHO_DISTANCE] = regs.holding[CMD_DISTANCE];
  regs.inputRegs[ECHO_POSITION] = regs.holding[CMD_POSITION];
  regs.inputRegs[ECHO_REQUEST_ID] = regs.holding[CMD_REQUEST_ID];
  regs.discrete[ECHO_FEED_CONVEYOR] = regs.coils[CMD_FEED_CONVEYOR];
  regs.discrete[ECHO_BRUSH_ON] = regs.coils[CMD_BRUSH_ON];

  // ---- watchdog: arms on FIRST heartbeat, never disarms; a resumed
  // heartbeat clears a trip. Before any orchestrator has spoken there is
  // nothing to supervise (a powered-but-idle line must not blast fans).
  uint16_t hb = regs.holding[REG_HEARTBEAT];
  if (hb != lastHeartbeat) {
    watchdogArmed = true;
    lastHeartbeat = hb;
    heartbeatSeenAtMs = nowMs;
    regs.inputRegs[REG_WATCHDOG_TRIPPED] = 0;
  } else if (watchdogArmed && (nowMs - heartbeatSeenAtMs) > WATCHDOG_TIMEOUT_MS) {
    regs.inputRegs[REG_WATCHDOG_TRIPPED] = 1;
  }
  bool tripped = regs.inputRegs[REG_WATCHDOG_TRIPPED] != 0;

  // ---- fans: relay outputs; FAIL ON when tripped (§7: a dead orchestrator
  // must never stop parts drying). Feedback mirrors the OUTPUT we drive —
  // truthful about the relay, not the airflow; a real airflow sensor can
  // replace it on these same registers later.
  bool ifFan = tripped || regs.holding[REG_F1_FAN] != 0;
  bool fdFan = tripped || regs.holding[REG_F2_FAN] != 0;
  PIN_F1_FAN.State(ifFan);
  PIN_F2_FAN.State(fdFan);
  regs.inputRegs[REG_F1_FAN_FB] = ifFan ? 1 : 0;
  regs.inputRegs[REG_F2_FAN_FB] = fdFan ? 1 : 0;

  // ---- shutter: double-solenoid 5/2 detented valve — the commanded side is
  // energised continuously, the opposite side off; de-energised it HOLDS
  // position (§7: shutter holds state on fault, through power loss included).
  // Feedback is the REAL end switches, never an echo — zone motion gates on it.
  bool wantOpen = regs.holding[REG_SH_CMD] != 0;
  PIN_SH_OPEN_SOL.State(wantOpen);
  PIN_SH_CLOSE_SOL.State(!wantOpen);
  if (shutOpen && !shutClosed) {
    regs.inputRegs[REG_SH_FB] = 1;
  } else if (shutClosed && !shutOpen) {
    regs.inputRegs[REG_SH_FB] = 0;
  } else {
    regs.inputRegs[REG_SH_FB] = 2;  // travelling (or switch fault)
  }

  // ---- zones
  tickZone(zone1, tripped);
  tickZone(zone2, tripped);

  // ---- feed conveyor (legacy M1 semantics): runs while the coil is set.
  IN_BELT.VelMax(velocityLimit());
  IN_BELT.AccelMax(accelLimit());
  if (!tripped && regs.coils[CMD_FEED_CONVEYOR]) {
    IN_BELT.MoveVelocity(velocityLimit());
  } else {
    IN_BELT.MoveStopDecel();
  }

  // ---- brush (legacy M2-role, now M3; legacy coil + tuned velocity).
  if (!tripped && regs.coils[CMD_BRUSH_ON]) {
    O_BRUSH.MoveVelocity(BRUSH_VELOCITY_SPS);
  } else {
    O_BRUSH.MoveStopDecel();
  }
}
