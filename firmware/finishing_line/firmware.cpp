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

ZoneBlock zone1 = {REG_ZONE1_MOTION_MODE, REG_ZONE1_DISTANCE, REG_ZONE1_REQUEST_ID,
                   REG_ZONE1_DIRECTION,   REG_ZONE1_TARGET,   REG_ZONE1_STATE,
                   REG_ZONE1_REQID_ACK,   &MOTOR_ZONE1};
ZoneBlock zone2 = {REG_ZONE2_MOTION_MODE, REG_ZONE2_DISTANCE, REG_ZONE2_REQUEST_ID,
                   REG_ZONE2_DIRECTION,   REG_ZONE2_TARGET,   REG_ZONE2_STATE,
                   REG_ZONE2_REQID_ACK,   &MOTOR_ZONE2};

// --------------------------------------------------------------- watchdog
uint16_t lastHeartbeat = 0;
uint32_t heartbeatSeenAtMs = 0;
bool watchdogArmed = false;

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
          case TARGET_IF_PRESENT: z.edgeSensorReg = REG_IF_PRESENT; break;
          case TARGET_S_PRESENT: z.edgeSensorReg = REG_S_PRESENT; break;
          case TARGET_FD_PRESENT: z.edgeSensorReg = REG_FD_PRESENT; break;
          case TARGET_HANDOFF_TO_Z1: z.edgeSensorReg = REG_HANDOFF_TO_Z1; break;
          case TARGET_HANDOFF_TO_Z2: z.edgeSensorReg = REG_HANDOFF_TO_Z2; break;
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
  // Motors: open-loop step/dir, legacy clocking and polarity (ino:192-207).
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_LOW);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
  MotorDriver *motors[] = {&MOTOR_ZONE1, &MOTOR_ZONE2, &MOTOR_FEED, &MOTOR_BRUSH};
  for (MotorDriver *m : motors) {
    m->VelMax(DEFAULT_VELOCITY_SPS);
    m->AccelMax(DEFAULT_ACCEL_SPS2);
    m->PolarityInvertSDEnable(INVERT_STEP_ENABLE);
    m->PolarityInvertSDDirection(INVERT_STEP_DIRECTION);
    m->EnableRequest(true);
  }
  MOTOR_BRUSH.VelMax(BRUSH_VELOCITY_SPS);
  MOTOR_BRUSH.AccelMax(BRUSH_ACCEL_SPS2);

  PIN_IF_PRESENT.Mode(Connector::INPUT_DIGITAL);
  PIN_S_PRESENT.Mode(Connector::INPUT_DIGITAL);
  PIN_FD_PRESENT.Mode(Connector::INPUT_DIGITAL);
  PIN_HANDOFF_TO_Z2.Mode(Connector::INPUT_DIGITAL);
  PIN_HANDOFF_TO_Z1.Mode(Connector::INPUT_DIGITAL);
  PIN_INQ_PRESENT.Mode(Connector::INPUT_DIGITAL);
  PIN_OUT_PRESENT.Mode(Connector::INPUT_DIGITAL);
  PIN_SHUTTER_OPEN_SW.Mode(Connector::INPUT_DIGITAL);
  PIN_SHUTTER_CLOSED_SW.Mode(Connector::INPUT_DIGITAL);
  PIN_IF_FAN.Mode(Connector::OUTPUT_DIGITAL);
  PIN_FD_FAN.Mode(Connector::OUTPUT_DIGITAL);
  PIN_SHUTTER_OPEN_SOL.Mode(Connector::OUTPUT_DIGITAL);
  PIN_SHUTTER_CLOSE_SOL.Mode(Connector::OUTPUT_DIGITAL);

  // Static IP — see io_map.h for why DHCP is deliberately gone.
  IPAddress ip(CC_IP_ADDRESS);
  IPAddress gw(CC_GATEWAY);
  IPAddress mask(CC_NETMASK);
  uint8_t mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE};  // legacy MAC (ino:27)
  Ethernet.begin(mac, ip, gw, gw, mask);
  while (Ethernet.linkStatus() == LinkOFF) {
    delay(500);  // no cable, nothing to do — same posture as legacy
  }
  server.begin();

  regs.inputRegs[REG_SERVER_STATE] = STATE_READY;  // legacy observability
  regs.inputRegs[REG_ZONE1_STATE] = STATE_READY;
  regs.inputRegs[REG_ZONE2_STATE] = STATE_READY;
  regs.holding[REG_ZONE1_MOTION_MODE] = MODE_IDLE;
  regs.holding[REG_ZONE2_MOTION_MODE] = MODE_IDLE;
}

void firmwareLoop() {
  uint32_t nowMs = Milliseconds();
  server.poll();

  // ---- sensors -> discrete inputs (debounced; these gate belt stops)
  regs.discrete[REG_IF_PRESENT] = dbIf.update(PIN_IF_PRESENT.State(), nowMs);
  regs.discrete[REG_S_PRESENT] = dbS.update(PIN_S_PRESENT.State(), nowMs);
  regs.discrete[REG_FD_PRESENT] = dbFd.update(PIN_FD_PRESENT.State(), nowMs);
  regs.discrete[REG_HANDOFF_TO_Z1] = dbHandZ1.update(PIN_HANDOFF_TO_Z1.State(), nowMs);
  regs.discrete[REG_HANDOFF_TO_Z2] = dbHandZ2.update(PIN_HANDOFF_TO_Z2.State(), nowMs);
  regs.discrete[REG_INQ_PRESENT] = dbInq.update(PIN_INQ_PRESENT.State(), nowMs);
  regs.discrete[REG_OUT_PRESENT] = dbOut.update(PIN_OUT_PRESENT.State(), nowMs);
  bool shutOpen = dbShutOpen.update(PIN_SHUTTER_OPEN_SW.State(), nowMs);
  bool shutClosed = dbShutClosed.update(PIN_SHUTTER_CLOSED_SW.State(), nowMs);

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
  bool ifFan = tripped || regs.holding[REG_IF_FAN_CMD] != 0;
  bool fdFan = tripped || regs.holding[REG_FD_FAN_CMD] != 0;
  PIN_IF_FAN.State(ifFan);
  PIN_FD_FAN.State(fdFan);
  regs.inputRegs[REG_IF_FAN_FEEDBACK] = ifFan ? 1 : 0;
  regs.inputRegs[REG_FD_FAN_FEEDBACK] = fdFan ? 1 : 0;

  // ---- shutter: double-solenoid 5/2 detented valve — the commanded side is
  // energised continuously, the opposite side off; de-energised it HOLDS
  // position (§7: shutter holds state on fault, through power loss included).
  // Feedback is the REAL end switches, never an echo — zone motion gates on it.
  bool wantOpen = regs.holding[REG_SHUTTER_CMD] != 0;
  PIN_SHUTTER_OPEN_SOL.State(wantOpen);
  PIN_SHUTTER_CLOSE_SOL.State(!wantOpen);
  if (shutOpen && !shutClosed) {
    regs.inputRegs[REG_SHUTTER_FEEDBACK] = 1;
  } else if (shutClosed && !shutOpen) {
    regs.inputRegs[REG_SHUTTER_FEEDBACK] = 0;
  } else {
    regs.inputRegs[REG_SHUTTER_FEEDBACK] = 2;  // travelling (or switch fault)
  }

  // ---- zones
  tickZone(zone1, tripped);
  tickZone(zone2, tripped);

  // ---- feed conveyor (legacy M1 semantics): runs while the coil is set.
  MOTOR_FEED.VelMax(velocityLimit());
  MOTOR_FEED.AccelMax(accelLimit());
  if (!tripped && regs.coils[CMD_FEED_CONVEYOR]) {
    MOTOR_FEED.MoveVelocity(velocityLimit());
  } else {
    MOTOR_FEED.MoveStopDecel();
  }

  // ---- brush (legacy M2-role, now M3; legacy coil + tuned velocity).
  if (!tripped && regs.coils[CMD_BRUSH_ON]) {
    MOTOR_BRUSH.MoveVelocity(BRUSH_VELOCITY_SPS);
  } else {
    MOTOR_BRUSH.MoveStopDecel();
  }
}
