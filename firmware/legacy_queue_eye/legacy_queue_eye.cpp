/*
  ModbusTCP Server for ClearCore Arduino wrapper

  (c)2021 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266

  This code is licensed under the BSD New License. See LICENSE.txt for more info.

  ============================================================================
  LEGACY v1.1 (2026-07-24) — a COPY of the untouched rollback artifact
  (../../modbustest.ino, whose ino:NN line numbers cell-config references).
  Exactly three deltas, all additive / rollback-compatible (the legacy
  Polyscope program reads none of them):

    1. AUX eye on DI-6 (F18, NPN), an input-only pin, served as discrete
       input 7. The firmware just serves the raw pin; what the eye WATCHES
       lives PC-side. Since 2026-07-25 it is the STAGING eye (~450 mm past
       the feed junction — the choreography's park sensor); it briefly
       watched the infeed queue before that.
    2. STATIC IP 192.168.1.18: the rollback program (script:20) and the PC
       sequencer both hardcode this address; DHCP was the documented .17/.18
       trap (cell-config discrepancies). NEVER power this board and the
       original DHCP board on the same LAN simultaneously.
    3. This banner.
    4. A one-line serial boot print (the v1.0 static path was silent).

  Build: pio run -e legacy_queue_eye   (see ../../platformio.ini)
  ============================================================================
*/

#include <Arduino.h> // v1.1: built as .cpp (PlatformIO), not an .ino sketch

#include <Ethernet.h> // Ethernet library v2 is required

// v1.1: prototypes the Arduino ino-preprocessor used to auto-generate.
String motionModeToString(int motionInt);
void MoveDistance(int distance);
int getDirectionSign();
String getStateString();
void printState();
void updateLocals();
int inchesToSteps(float inches);

#include <ModbusAPI.h>
#include <ModbusTCPTemplate.h>

#include "ClearCore.h"

class ModbusEthernet : public ModbusAPI<ModbusTCPTemplate<EthernetServer, EthernetClient>>
{
};

EthernetTcpServer server = EthernetTcpServer(8888);

const uint16_t REG = 512;       // Modbus Hreg Offset
const int32_t showDelay = 5000; // Show result every n'th mellisecond

bool usingDhcp = false;                            // v1.1: deterministic address (delta 2)
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE}; // MAC address for your controller
IPAddress ip(192, 168, 1, 18);                     // v1.1: the address everything hardcodes
ModbusEthernet mb;                                 // Declare ModbusTCP instance

// Callback function for client connect. Returns true to allow connection.
bool cbConn(IPAddress ip)
{
  Serial.println(ip);
  return true;
}

int counter = 0;
int loopLength = 5;

// Specifies which motor to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motor ConnectorM0      // main conveyor
#define cpMotor ConnectorM1    // feed conveyor
#define brushMotor ConnectorM2 // spray cleanoff brush

// int velocityLimit = 10000;      // pulses per sec
// int accelerationLimit = 100000; // pulses per sec^2
// bool direction = 1;             // 0 for reverse, 1 for forawrd
// int targetDistance = 0;         // for distance based moves
// String motionMode = "distance"; // 0 for distance, 1 for position, 2 for continuous, 3 for idle
// int targetPosition = 0;         // for position based moves
// int currentPosition = 0;        // track current position
// int requestId = 0;              // request id for differentiating new requests
// int currentRequestId = 1;       // request id holder to compare incoming jobs
// bool feedConveyorOn = false;
// bool brushMotorOn = false;
// bool robotSending = false;

String stateString = getStateString();
String currentStateString = getStateString();

#define workZeroPin IO0
PinStatus workAtZero;
#define offLoadPin IO1
PinStatus offLoadStatus;
#define onLoadPin IO2
PinStatus onLoadStatus;
#define stagingPin DI6 // v1.1 (delta 1): aux eye (staging duty) — input-only pin, IO3-5 stay free
PinStatus stagingStatus;

// to make sure that server gets all commands before starting a move
//  there will be matching "remote" and "local" values

// Params: set by client. Hreg Offsets
#define MOTION_MODE_REMOTE 100
#define DIRECTION_REMOTE 101
#define VELOCITY_REMOTE 102
#define ACCELERATION_REMOTE 103
#define DISTANCE_REMOTE 104
#define POSITION_REMOTE 105
#define REQUESTID_REMOTE 106
#define FEEDCONVEYOR_REMOTE 107
#define BRUSH_ON_REMOTE 108

#define MOTION_MODE_LOCAL 200
#define DIRECTION_LOCAL 201
#define VELOCITY_LOCAL 202
#define ACCELERATION_LOCAL 203
#define DISTANCE_LOCAL 204
#define POSITION_LOCAL 205
#define REQUESTID_LOCAL 206
#define FEEDCONVEYOR_LOCAL 207
#define BRUSH_ON_LOCAL 208

#define STATE 1
#define JOB 2
#define RUN 3
#define WORKZERO 4
#define OFFLOAD 5
#define ONLOAD 6
#define STAGING 7 // v1.1 (delta 1): DI-6 aux eye (staging duty since 2026-07-25)

#define velocityLimit mb.Ireg(VELOCITY_LOCAL)
#define accelerationLimit mb.Ireg(ACCELERATION_LOCAL)
#define direction mb.Ists(DIRECTION_LOCAL)
#define motionMode mb.Ireg(MOTION_MODE_LOCAL)
#define targetPosition mb.Ireg(POSITION_LOCAL)
#define targetDistance mb.Ireg(DISTANCE_LOCAL)
#define feedConveyorOn mb.Ists(FEEDCONVEYOR_LOCAL)
#define brushMotorOn mb.Ists(BRUSH_ON_LOCAL)
#define newRequestId mb.Ireg(REQUESTID_LOCAL)
#define run mb.Ireg(RUN);

int currentPosition = 0;        // track current position
int oldRequestId = 0;              // request id for differentiating new requests

void setup()
{
  Serial.begin(9600);
  uint32_t timeout = 5000;
  uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout)
    continue;

  // Get the Ethernet mosdule up and running.
  if (usingDhcp)
  {
    int dhcpSuccess = Ethernet.begin(mac);
    if (dhcpSuccess)
    {
      Serial.println("DHCP configuration was successful.");
      Serial.println(Ethernet.localIP());
    }
    else
    {
      Serial.println("DHCP configuration was unsuccessful!");
      Serial.println("Try again using a manual configuration...");
      while (true)
        continue;
    }
  }
  else
  {
    // v1.1 (delta 2): the v1.0 two-arg begin(mac, ip) was DEAD CODE — never
    // exercised in production (DHCP was always on). Use the explicit five-arg
    // form, the exact call the rewrite firmware proved on this hardware.
    // (A dark-LED scare during bring-up was a bad cable, not this call.)
    IPAddress gw(192, 168, 1, 1);
    IPAddress mask(255, 255, 255, 0);
    Ethernet.begin(mac, ip, gw, gw, mask);
    // v1.1 (delta 4): the v1.0 static path booted silently — say who we are,
    // so a serial monitor can distinguish this firmware at a glance.
    Serial.println("legacy v1.1 — static 192.168.1.18, DI-6 eye @ discrete 7 (staging)");
  }

  // Make sure the physical link is up before continuing.
  while (Ethernet.linkStatus() == LinkOFF)
  {
    Serial.println("The Ethernet cable is unplugged...");
    delay(1000);
  }

  // Start listening for TCP connections on port 8888.
  server.Begin();

  mb.server(); // Act as Modbus TCP server
  mb.onConnect(cbConn);

  // Conveyor state: read by client
  // 0 Not ready
  // 1 Ready
  // 2 Moving

  mb.addIreg(STATE, 0); // Ireg 1 communicates server state, starting with not ready
  mb.addIreg(JOB, 1);
  // Ireg 2 communicates user selected job: 1 - cube, 2 - browser,  3 - 45
  // 4 - stereocab 2, 5 - sc 3, 6 - sc4
  mb.addIsts(RUN, 0); // start / stop control for conveyor
  mb.addIsts(WORKZERO, 0);   // is the work piece at "zero" npn sensor io0
  mb.addIsts(OFFLOAD, 0);    // is the offload npn sensor io1 active
  mb.addIsts(ONLOAD, 0);     // is the center onloan npn sensor io2 active
  mb.addIsts(STAGING, 0);    // v1.1 (delta 1): DI-6 aux eye (staging)

  mb.addHreg(MOTION_MODE_REMOTE);  // motion mode
  mb.addCoil(DIRECTION_REMOTE);    // direction
  mb.addHreg(VELOCITY_REMOTE);     // velocity
  mb.addHreg(ACCELERATION_REMOTE); // acceleration
  mb.addHreg(DISTANCE_REMOTE);     // distance
  mb.addHreg(POSITION_REMOTE);     // position
  mb.addHreg(REQUESTID_REMOTE);    // request id
  mb.addCoil(FEEDCONVEYOR_REMOTE); // feed conveyor active?
  mb.addCoil(BRUSH_ON_REMOTE);     // is the robot telling cc to turn on brush motor

  mb.addIreg(MOTION_MODE_LOCAL, 3);  // motion mode
  mb.addIsts(DIRECTION_LOCAL);    // direction
  mb.addIreg(VELOCITY_LOCAL);     // velocity
  mb.addIreg(ACCELERATION_LOCAL); // acceleration
  mb.addIreg(DISTANCE_LOCAL);     // distance
  mb.addIreg(POSITION_LOCAL);     // position
  mb.addIreg(REQUESTID_LOCAL);    // request id
  mb.addIsts(FEEDCONVEYOR_LOCAL); // feed conveyor active?
  mb.addIsts(BRUSH_ON_LOCAL);     // is the robot telling cc to turn on brush motor

  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_LOW);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
  motor.VelMax(velocityLimit);
  motor.AccelMax(accelerationLimit);
  motor.PolarityInvertSDEnable(true);
  motor.PolarityInvertSDDirection(false);
  motor.EnableRequest(true);

  cpMotor.VelMax(velocityLimit);
  cpMotor.AccelMax(accelerationLimit);
  cpMotor.PolarityInvertSDEnable(true);
  // cpMotor.PolarityInvertSDDirection(true);
  cpMotor.EnableRequest(true);

  brushMotor.PolarityInvertSDEnable(true);
  brushMotor.EnableRequest(true);
  brushMotor.AccelMax(100000);

  // todo: "home" the conveyor?
  // will need a sensor to indicate that a new workpiece has enterred the work area
  // will need a sensor to confirm that a workpiece has exited the work area
}

void loop()
{
  workAtZero = digitalRead(workZeroPin);
  if (workAtZero)
  {
    mb.Ists(WORKZERO, 1);
  }
  else
  {
    mb.Ists(WORKZERO, 0);
  }

  offLoadStatus = digitalRead(offLoadPin);
  if (offLoadStatus)
  {
    mb.Ists(OFFLOAD, 1);
  }
  else
  {
    mb.Ists(OFFLOAD, 0);
  }

  onLoadStatus = digitalRead(onLoadPin);
  if (onLoadStatus)
  {
    mb.Ists(ONLOAD, 1);
  }
  else
  {
    mb.Ists(ONLOAD, 0);
  }

  // v1.1 (delta 1): the DI-6 aux eye, same raw-read pattern as the others.
  // Polarity (F18 = part present reads LOW) is normalized PC-side via
  // line-config legacy_mode.sensor_polarity, like the OFFLOAD F18 swap.
  stagingStatus = digitalRead(stagingPin);
  if (stagingStatus)
  {
    mb.Ists(STAGING, 1);
  }
  else
  {
    mb.Ists(STAGING, 0);
  }

  // Obtain a reference to a connected client with incoming data available.
  EthernetTcpClient client = server.Available();
  if (client.Connected())
  {
    String readString = "";

    // The server has returned a connected client with incoming data available.
    while (client.BytesAvailable() > 0)
    {
      char c = client.Read();
      readString += c;
    }

    Serial.println(readString);
    if (readString == "go")
    {
      mb.Ists(RUN, 1);
      client.Send("Starting conveyor");
    }
    else if (readString == "stop")
    {
      mb.Ists(RUN, 0);
      client.Send("Stopping conveyor");
    }
    else
    {
      mb.Ireg(2, readString.toInt());
      client.Send("Job Updated");
    }
  }

  // update motor params via modbus
  mb.task();
  updateLocals();

  motor.VelMax(velocityLimit);
  motor.AccelMax(accelerationLimit);
  cpMotor.VelMax(velocityLimit);
  cpMotor.AccelMax(accelerationLimit);

  if (motionModeToString(motionMode)== "idle")
  {
    motor.MoveStopDecel();
    mb.Ireg(STATE, 1);
  }

  else if (motionModeToString(motionMode)== "continuous")
  {
    motor.MoveVelocity(velocityLimit * getDirectionSign());
    mb.Ireg(STATE, 2);
  }

  else if (motionModeToString(motionMode)== "position" && targetPosition != currentPosition)
  {
    mb.Ireg(STATE, 2);
    MoveDistance(targetPosition - currentPosition);
  }

  else if (motionModeToString(motionMode)== "distance" && newRequestId != oldRequestId)
  {
    mb.Ireg(STATE, 2);
    MoveDistance(targetDistance * getDirectionSign());
    oldRequestId = newRequestId;
  }

  else if (motionModeToString(motionMode)== "continuous")
  {
    mb.Ireg(STATE, 2);
    motor.MoveVelocity(velocityLimit * getDirectionSign());
  }

  // turn on the feed conveyor to load the next part
  if (feedConveyorOn)
  {
    cpMotor.MoveVelocity(velocityLimit);
  }
  else
  {
    cpMotor.MoveStopDecel();
  }

  if (brushMotorOn)
  {
    brushMotor.MoveVelocity(10000);
  }
  else
  {
    brushMotor.MoveStopDecel();
  }

  if (motionModeToString(motionMode) == "distance" || motionModeToString(motionMode) == "position")
  {
    if (motor.StepsComplete())
    {
      mb.Ireg(STATE, 1);
    }
  }
}

String motionModeToString(int motionInt)
{
  switch (motionInt)
  {
  case 0:
    return "distance";
    break;
  case 1:
    return "position";
    break;
  case 2:
    return "continuous";
    break;
  case 3:
    return "idle";
    break;
  default:
    return "idle";
  }
}

void MoveDistance(int distance)
{
  motor.Move(distance);
}

int getDirectionSign()
{
  if (direction == true)
    return 1;
  else
    return -1;
}

String getStateString()
{
  return "stateString";
}

void printState()
{
  Serial.print("Server state: ");
  Serial.println(mb.Ireg(STATE));
  Serial.print("Direction: ");
  Serial.println(direction);
  Serial.print("Velocity: ");
  Serial.println(velocityLimit);
  Serial.print("Acceleration: ");
  Serial.println(accelerationLimit);
  Serial.print("Current position: ");
  Serial.println(currentPosition);
  Serial.print("Target Distance: ");
  Serial.println(targetDistance);
  Serial.print("Target Position: ");
  Serial.println(targetPosition);
  Serial.print("Motion Mode: ");
  Serial.println(motionMode);
  Serial.print("New RequestId: ");
  Serial.println(newRequestId);
  Serial.print("Old RequestId: ");
  Serial.println(oldRequestId);
  Serial.print("Run: ");
  Serial.println(oldRequestId);
  Serial.print("Old RequestId: ");
  Serial.println(oldRequestId);
  Serial.println("");
  // motor status
}

void updateLocals(){
  mb.Ireg(VELOCITY_LOCAL, mb.Hreg(VELOCITY_REMOTE));
  mb.Ireg(ACCELERATION_LOCAL, mb.Hreg(ACCELERATION_REMOTE));
  mb.Ireg(MOTION_MODE_LOCAL, mb.Hreg(MOTION_MODE_REMOTE));  // motion mode
  mb.Ists(DIRECTION_LOCAL, mb.Coil(DIRECTION_REMOTE));    // direction
  mb.Ireg(DISTANCE_LOCAL, mb.Hreg(DISTANCE_REMOTE));     // distance
  mb.Ireg(POSITION_LOCAL, mb.Hreg(POSITION_REMOTE));     // position
  mb.Ireg(REQUESTID_LOCAL, mb.Hreg(REQUESTID_REMOTE));    // request id
  mb.Ists(FEEDCONVEYOR_LOCAL, mb.Coil(FEEDCONVEYOR_REMOTE)); // feed conveyor active?
  mb.Ists(BRUSH_ON_LOCAL, mb.Coil(BRUSH_ON_REMOTE));     // is the robot telling cc to turn on brush motor
}

int inchesToSteps(float inches)
{
  // diameter  = 1.710
  // radius = .88
  // circumference = 5.37212
  // inches per step = 5.37212 / 200 Steps
  float c = 5.37212;
  int steps = 200;
  float ips = c / steps;
  float spi = steps / c;
  return inches * spi;
}