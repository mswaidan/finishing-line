// Modbus register map — C++ twin of src/finishing_line/devices/registers.py.
//
// THAT FILE IS THE SOURCE OF TRUTH. If an address changes here it must change
// there (and in the fake, and the driver tests will catch the disagreement).
// Naming follows the ROBOT-side convention of the legacy program, matching
// cell-config.yaml: the 100-block is commands written BY the master, the
// 200-block is the echo the firmware writes back.

#pragma once

#include <stdint.h>

// Legacy status block (served for observability; SERVER_STATE static READY).
enum StatusReg : uint16_t {
  REG_SERVER_STATE = 1,
  REG_JOB = 2,
  REG_RUN = 3,
  REG_WORK_AT_ZERO = 4,
  REG_OFFLOAD = 5,
  REG_ONLOAD = 6,
};

// Legacy command block (holding registers / coils, written by the master).
enum CommandReg : uint16_t {
  CMD_MOTION_MODE = 100,
  CMD_DIRECTION = 101,   // coil
  CMD_VELOCITY = 102,
  CMD_ACCELERATION = 103,
  CMD_DISTANCE = 104,
  CMD_POSITION = 105,
  CMD_REQUEST_ID = 106,
  CMD_FEED_CONVEYOR = 107,  // coil — ACTIVELY USED: the IN queue belt
  CMD_BRUSH_ON = 108,       // coil
};

// Legacy echo block (input registers / discrete inputs, firmware-written).
enum EchoReg : uint16_t {
  ECHO_MOTION_MODE = 200,
  ECHO_DIRECTION = 201,
  ECHO_VELOCITY = 202,
  ECHO_ACCELERATION = 203,
  ECHO_DISTANCE = 204,
  ECHO_POSITION = 205,
  ECHO_REQUEST_ID = 206,
  ECHO_FEED_CONVEYOR = 207,
  ECHO_BRUSH_ON = 208,
};

// Rewrite block — see registers.py class New for full semantics.
enum NewReg : uint16_t {
  REG_SH_CMD = 300,       // 0 closed, 1 open
  REG_F1_FAN = 301,
  REG_F2_FAN = 302,

  REG_Z1_MODE = 310,
  REG_Z1_DIST = 311,
  REG_Z1_REQID = 312,
  REG_Z1_DIR = 313,   // coil: 1 = downstream
  REG_Z1_TARGET = 314,      // SensorTarget encoding, MODE_SENSOR_STOP
  REG_Z2_MODE = 320,
  REG_Z2_DIST = 321,
  REG_Z2_REQID = 322,
  REG_Z2_DIR = 323,   // coil
  REG_Z2_TARGET = 324,

  REG_HEARTBEAT = 330,         // watchdog arms on FIRST change, never disarms

  REG_SH_FB = 400,  // 0 closed, 1 open, 2 moving/unknown (SENSED)
  REG_F1_FAN_FB = 401,
  REG_F2_FAN_FB = 402,
  REG_F1_EYE = 403,        // discrete
  REG_O_EYE = 404,         // discrete
  REG_F2_EYE = 405,        // discrete
  REG_IN_COUNT = 406,         // no count sensor; see IN_EYE (409)
  REG_Z2_EYE = 407,     // discrete
  REG_Z1_EYE = 408,     // discrete
  REG_IN_EYE = 409,       // discrete: queue-head eye
  REG_Z1_STATE = 410,       // 0 not ready, 1 ready, 2 moving
  REG_Z2_STATE = 411,
  REG_WATCHDOG_TRIPPED = 412,
  REG_Z1_ACK = 413,   // last RECOGNISED request id (the move is running)
  REG_Z2_ACK = 414,
  REG_OUT_EYE = 415,       // discrete: outfeed occupancy eye
};

// Zone motion modes (legacy vocabulary + sensor-stop).
enum MotionMode : uint16_t {
  MODE_DISTANCE = 0,
  MODE_POSITION = 1,
  MODE_CONTINUOUS = 2,
  MODE_IDLE = 3,
  MODE_SENSOR_STOP = 4,
};

// ZONE*_TARGET encoding: low bits pick the sensor, +8 = falling edge.
// EDGES, not levels: firmware records the level at arm time and stops on the
// first TRANSITION to the target polarity (see fake_clearcore.py).
enum SensorTargetCode : uint16_t {
  TARGET_F1_EYE = 1,
  TARGET_O_EYE = 2,
  TARGET_F2_EYE = 3,
  TARGET_Z1_EYE = 4,
  TARGET_Z2_EYE = 5,
  TARGET_FALLING_FLAG = 8,
};

enum ZoneState : uint16_t {
  STATE_NOT_READY = 0,
  STATE_READY = 1,
  STATE_MOVING = 2,
};
