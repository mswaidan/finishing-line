// Minimal Modbus TCP server — C++ twin of the fake's _handle_pdu.
//
// Hand-rolled on purpose: the driver needs exactly FC 1/2/3/4/5/6/16, the
// whole protocol is a 7-byte MBAP header plus a tiny PDU, and owning it means
// the firmware has no third-party Modbus dependency and byte-for-byte matches
// the executable spec in sim/fake_clearcore.py.

#pragma once

#include <stdint.h>

#include "Ethernet.h"

// Register table sizes (uint16 regs ~4 KB, bools ~2 KB — trivial for the
// ClearCore's 256 KB RAM).
constexpr uint16_t TABLE_SIZE = 1024;

struct RegisterFile {
  bool coils[TABLE_SIZE] = {};
  bool discrete[TABLE_SIZE] = {};
  uint16_t holding[TABLE_SIZE] = {};
  uint16_t inputRegs[TABLE_SIZE] = {};
};

class ModbusTcpServer {
 public:
  explicit ModbusTcpServer(RegisterFile &regs) : _regs(regs), _server(MODBUS_PORT) {}

  void begin();
  // Service pending requests; call every loop. Non-blocking.
  void poll();

  static constexpr uint16_t MODBUS_PORT = 502;

 private:
  static constexpr int MAX_CLIENTS = 4;
  static constexpr int BUF_SIZE = 300;  // MBAP(7) + max PDU comfortably

  struct ClientSlot {
    EthernetClient client;
    uint8_t buf[BUF_SIZE];
    int have = 0;
  };

  void _service(ClientSlot &slot);
  // Returns response PDU length written into out; 0 = drop the request.
  int _handlePdu(const uint8_t *pdu, int len, uint8_t *out);

  RegisterFile &_regs;
  EthernetServer _server;
  ClientSlot _slots[MAX_CLIENTS];
};
