#include "modbus_tcp.h"

namespace {
constexpr uint8_t FC_READ_COILS = 1;
constexpr uint8_t FC_READ_DISCRETE = 2;
constexpr uint8_t FC_READ_HOLDING = 3;
constexpr uint8_t FC_READ_INPUT = 4;
constexpr uint8_t FC_WRITE_COIL = 5;
constexpr uint8_t FC_WRITE_REGISTER = 6;
constexpr uint8_t FC_WRITE_REGISTERS = 16;

inline uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
inline void putBe16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFF);
}
}  // namespace

void ModbusTcpServer::begin() { _server.begin(); }

void ModbusTcpServer::poll() {
  // Adopt any newly connected client into a free slot.
  EthernetClient fresh = _server.accept();
  if (fresh) {
    for (auto &slot : _slots) {
      if (!slot.client.connected()) {
        slot.client = fresh;
        slot.have = 0;
        fresh = EthernetClient();
        break;
      }
    }
    if (fresh) fresh.stop();  // all slots busy
  }
  for (auto &slot : _slots) {
    if (slot.client.connected()) _service(slot);
  }
}

void ModbusTcpServer::_service(ClientSlot &slot) {
  while (slot.client.available() > 0 && slot.have < BUF_SIZE) {
    int c = slot.client.read();
    if (c < 0) break;
    slot.buf[slot.have++] = (uint8_t)c;
  }

  // Process every complete frame in the buffer. MBAP: tid(2) pid(2) len(2)
  // unit(1); len counts unit + PDU.
  while (slot.have >= 7) {
    uint16_t length = be16(slot.buf + 4);
    int frame = 6 + length;
    if (length < 2 || frame > BUF_SIZE) {  // malformed; drop the connection
      slot.client.stop();
      slot.have = 0;
      return;
    }
    if (slot.have < frame) return;  // partial frame; wait for more bytes

    uint8_t out[BUF_SIZE];
    int outLen = _handlePdu(slot.buf + 7, length - 1, out + 7);
    if (outLen > 0) {
      // Reuse tid/pid/unit from the request.
      out[0] = slot.buf[0];
      out[1] = slot.buf[1];
      out[2] = slot.buf[2];
      out[3] = slot.buf[3];
      putBe16(out + 4, (uint16_t)(outLen + 1));
      out[6] = slot.buf[6];
      slot.client.write(out, (size_t)(7 + outLen));
    }
    // Shift any trailing bytes down (pipelined requests).
    slot.have -= frame;
    memmove(slot.buf, slot.buf + frame, (size_t)slot.have);
  }
}

int ModbusTcpServer::_handlePdu(const uint8_t *pdu, int len, uint8_t *out) {
  if (len < 1) return 0;
  uint8_t fc = pdu[0];

  if ((fc == FC_READ_COILS || fc == FC_READ_DISCRETE) && len >= 5) {
    uint16_t addr = be16(pdu + 1), count = be16(pdu + 3);
    if (count == 0 || count > 2000 || addr + count > TABLE_SIZE) goto illegal;
    {
      const bool *table = (fc == FC_READ_COILS) ? _regs.coils : _regs.discrete;
      uint8_t nbytes = (uint8_t)((count + 7) / 8);
      out[0] = fc;
      out[1] = nbytes;
      for (int i = 0; i < nbytes; i++) out[2 + i] = 0;
      for (uint16_t i = 0; i < count; i++) {
        if (table[addr + i]) out[2 + i / 8] |= (uint8_t)(1 << (i % 8));
      }
      return 2 + nbytes;
    }
  }
  if ((fc == FC_READ_HOLDING || fc == FC_READ_INPUT) && len >= 5) {
    uint16_t addr = be16(pdu + 1), count = be16(pdu + 3);
    if (count == 0 || count > 125 || addr + count > TABLE_SIZE) goto illegal;
    {
      const uint16_t *table = (fc == FC_READ_HOLDING) ? _regs.holding : _regs.inputRegs;
      out[0] = fc;
      out[1] = (uint8_t)(count * 2);
      for (uint16_t i = 0; i < count; i++) putBe16(out + 2 + 2 * i, table[addr + i]);
      return 2 + count * 2;
    }
  }
  if (fc == FC_WRITE_COIL && len >= 5) {
    uint16_t addr = be16(pdu + 1), value = be16(pdu + 3);
    if (addr >= TABLE_SIZE) goto illegal;
    _regs.coils[addr] = (value == 0xFF00);
    memcpy(out, pdu, 5);
    return 5;
  }
  if (fc == FC_WRITE_REGISTER && len >= 5) {
    uint16_t addr = be16(pdu + 1);
    if (addr >= TABLE_SIZE) goto illegal;
    _regs.holding[addr] = be16(pdu + 3);
    memcpy(out, pdu, 5);
    return 5;
  }
  if (fc == FC_WRITE_REGISTERS && len >= 6) {
    uint16_t addr = be16(pdu + 1), count = be16(pdu + 3);
    if (count == 0 || count > 123 || addr + count > TABLE_SIZE || len < 6 + 2 * count)
      goto illegal;
    for (uint16_t i = 0; i < count; i++) _regs.holding[addr + i] = be16(pdu + 6 + 2 * i);
    memcpy(out, pdu, 5);
    return 5;
  }

illegal:
  out[0] = (uint8_t)(fc | 0x80);
  out[1] = 0x01;  // illegal function / address
  return 2;
}
