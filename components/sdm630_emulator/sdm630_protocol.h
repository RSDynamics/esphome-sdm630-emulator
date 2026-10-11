#pragma once
// Pure protocol logic (no ESPHome dependencies) so it can be unit tested on a PC.
//
// Emulates the part of an Eastron SDM630 Modbus map that an EV charger needs for
// dynamic load balancing: per-phase voltage (V), current (A) and active power (W),
// each a big-endian IEEE-754 float32 spread over two 16-bit input registers.
//
// Register addresses are 0-based, as they appear on the wire.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace esphome {
namespace sdm630_emulator {

static constexpr size_t MBAP_HEADER_LEN = 7;       // transaction(2) protocol(2) length(2) unit(1)
static constexpr size_t MAX_FRAME_LEN = 260;       // 6 + max length field (254)
static constexpr size_t MAX_RESPONSE_LEN = 264;    // 7 + 2 + 2 * 125
static constexpr uint16_t MAX_READ_REGISTERS = 125;
static constexpr uint16_t REGISTER_LIMIT = 0x0200;  // SDM630 map ends well below this

static constexpr uint8_t FC_READ_HOLDING = 0x03;
static constexpr uint8_t FC_READ_INPUT = 0x04;

static constexpr uint8_t EX_ILLEGAL_FUNCTION = 0x01;
static constexpr uint8_t EX_ILLEGAL_ADDRESS = 0x02;
static constexpr uint8_t EX_ILLEGAL_VALUE = 0x03;
static constexpr uint8_t EX_DEVICE_FAILURE = 0x04;

struct Values {
  float voltage[3];    // V
  float current[3];    // A, signed: import positive, export negative
  float power[3];      // W, signed: import positive, export negative
  float frequency;     // Hz
};

// Combine raw sensor readings into the values served over Modbus.
// NaN inputs (sensor never reported, e.g. L2/L3 on a single-phase connection) become 0 W,
// and a missing/implausible voltage falls back to the nominal voltage.
inline Values compute_values(const float volt[3], const float delivered[3], const float returned[3],
                             float nominal_voltage, float frequency = 50.0f) {
  Values v{};
  v.frequency = frequency;
  for (int i = 0; i < 3; i++) {
    float u = volt[i];
    if (std::isnan(u) || u < 50.0f) u = nominal_voltage;
    float pd = std::isnan(delivered[i]) ? 0.0f : delivered[i];
    float pr = std::isnan(returned[i]) ? 0.0f : returned[i];
    float p = pd - pr;
    v.voltage[i] = u;
    v.power[i] = p;
    v.current[i] = p / u;
  }
  return v;
}

// Which float32 lives at an (even) base address.
inline float float_at(uint16_t base, const Values &v) {
  switch (base) {
    case 0x0000: return v.voltage[0];
    case 0x0002: return v.voltage[1];
    case 0x0004: return v.voltage[2];
    case 0x0006: return v.current[0];
    case 0x0008: return v.current[1];
    case 0x000A: return v.current[2];
    case 0x000C: return v.power[0];
    case 0x000E: return v.power[1];
    case 0x0010: return v.power[2];
    case 0x0034: return v.power[0] + v.power[1] + v.power[2];  // total system power
    case 0x0046: return v.frequency;
    default: return 0.0f;
  }
}

// One 16-bit register (works for any alignment of the request).
inline uint16_t register_word(uint16_t addr, const Values &v) {
  float f = float_at(static_cast<uint16_t>(addr & ~1u), v);
  uint32_t bits;
  std::memcpy(&bits, &f, sizeof(bits));
  return (addr & 1u) ? static_cast<uint16_t>(bits & 0xFFFFu) : static_cast<uint16_t>(bits >> 16);
}

// Length of the complete frame at the start of a TCP stream buffer.
//   >0 : total frame length (the caller may still need more bytes)
//    0 : not enough bytes yet to know
//   -1 : invalid header, drop the connection
inline int frame_length(const uint8_t *buf, size_t n) {
  if (n < 6) return 0;
  uint16_t protocol = static_cast<uint16_t>((buf[2] << 8) | buf[3]);
  uint16_t len = static_cast<uint16_t>((buf[4] << 8) | buf[5]);
  if (protocol != 0 || len < 2 || len > 254) return -1;
  return static_cast<int>(6 + len);
}

inline size_t build_exception(const uint8_t *req, uint8_t fc, uint8_t code, uint8_t *out) {
  out[0] = req[0];
  out[1] = req[1];
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = 3;
  out[6] = req[6];
  out[7] = static_cast<uint8_t>(fc | 0x80);
  out[8] = code;
  return 9;
}

// Handle one complete request frame. Returns the response length, or 0 for "do not reply".
// `values == nullptr` means the data is stale: every read is answered with a device failure
// exception so the charger falls back to its own safe current instead of using old numbers.
inline size_t build_response(const uint8_t *req, size_t req_len, const Values *values, uint8_t unit_id,
                             uint8_t *out) {
  if (req_len < MBAP_HEADER_LEN + 1) return 0;
  uint8_t unit = req[6];
  if (unit != unit_id && unit != 0xFF) return 0;
  uint8_t fc = req[7];
  if (fc != FC_READ_HOLDING && fc != FC_READ_INPUT) return build_exception(req, fc, EX_ILLEGAL_FUNCTION, out);
  if (req_len < MBAP_HEADER_LEN + 5) return build_exception(req, fc, EX_ILLEGAL_VALUE, out);

  uint16_t addr = static_cast<uint16_t>((req[8] << 8) | req[9]);
  uint16_t count = static_cast<uint16_t>((req[10] << 8) | req[11]);
  if (count == 0 || count > MAX_READ_REGISTERS) return build_exception(req, fc, EX_ILLEGAL_VALUE, out);
  if (static_cast<uint32_t>(addr) + count > REGISTER_LIMIT) return build_exception(req, fc, EX_ILLEGAL_ADDRESS, out);
  if (values == nullptr) return build_exception(req, fc, EX_DEVICE_FAILURE, out);

  size_t data_len = static_cast<size_t>(count) * 2;
  out[0] = req[0];
  out[1] = req[1];
  out[2] = 0;
  out[3] = 0;
  uint16_t length = static_cast<uint16_t>(3 + data_len);  // unit + fc + byte count + data
  out[4] = static_cast<uint8_t>(length >> 8);
  out[5] = static_cast<uint8_t>(length & 0xFF);
  out[6] = req[6];
  out[7] = fc;
  out[8] = static_cast<uint8_t>(data_len);
  for (uint16_t i = 0; i < count; i++) {
    uint16_t w = register_word(static_cast<uint16_t>(addr + i), *values);
    out[9 + 2 * i] = static_cast<uint8_t>(w >> 8);
    out[10 + 2 * i] = static_cast<uint8_t>(w & 0xFF);
  }
  return 9 + data_len;
}

}  // namespace sdm630_emulator
}  // namespace esphome
