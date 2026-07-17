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
  CMD_FEED_CONVEYOR = 107,  // coil — ACTIVELY USED: the INQ queue belt
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
  REG_SHUTTER_CMD = 300,       // 0 closed, 1 open
  REG_IF_FAN_CMD = 301,
  REG_FD_FAN_CMD = 302,

  REG_ZONE1_MOTION_MODE = 310,
  REG_ZONE1_DISTANCE = 311,
  REG_ZONE1_REQUEST_ID = 312,
  REG_ZONE1_DIRECTION = 313,   // coil: 1 = downstream
  REG_ZONE1_TARGET = 314,      // SensorTarget encoding, MODE_SENSOR_STOP
  REG_ZONE2_MOTION_MODE = 320,
  REG_ZONE2_DISTANCE = 321,
  REG_ZONE2_REQUEST_ID = 322,
  REG_ZONE2_DIRECTION = 323,   // coil
  REG_ZONE2_TARGET = 324,

  REG_HEARTBEAT = 330,         // watchdog arms on FIRST change, never disarms

  REG_SHUTTER_FEEDBACK = 400,  // 0 closed, 1 open, 2 moving/unknown (SENSED)
  REG_IF_FAN_FEEDBACK = 401,
  REG_FD_FAN_FEEDBACK = 402,
  REG_IF_PRESENT = 403,        // discrete
  REG_S_PRESENT = 404,         // discrete
  REG_FD_PRESENT = 405,        // discrete
  REG_INQ_COUNT = 406,         // no count sensor; see INQ_PRESENT (409)
  REG_HANDOFF_TO_Z2 = 407,     // discrete
  REG_HANDOFF_TO_Z1 = 408,     // discrete
  REG_INQ_PRESENT = 409,       // discrete: queue-head eye
  REG_ZONE1_STATE = 410,       // 0 not ready, 1 ready, 2 moving
  REG_ZONE2_STATE = 411,
  REG_WATCHDOG_TRIPPED = 412,
  REG_ZONE1_REQID_ACK = 413,   // last RECOGNISED request id (the move is running)
  REG_ZONE2_REQID_ACK = 414,
  REG_OUT_PRESENT = 415,       // discrete: outfeed occupancy eye
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
  TARGET_IF_PRESENT = 1,
  TARGET_S_PRESENT = 2,
  TARGET_FD_PRESENT = 3,
  TARGET_HANDOFF_TO_Z1 = 4,
  TARGET_HANDOFF_TO_Z2 = 5,
  TARGET_FALLING_FLAG = 8,
};

enum ZoneState : uint16_t {
  STATE_NOT_READY = 0,
  STATE_READY = 1,
  STATE_MOVING = 2,
};
