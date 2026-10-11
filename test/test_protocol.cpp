#include <cassert>
#include <cstdio>
#include <vector>
#include "sdm630_protocol.h"
using namespace esphome::sdm630_emulator;

static std::vector<uint8_t> req(uint16_t tid, uint8_t unit, uint8_t fc, uint16_t addr, uint16_t count) {
  return {uint8_t(tid >> 8), uint8_t(tid), 0, 0, 0, 6, unit, fc, uint8_t(addr >> 8), uint8_t(addr), uint8_t(count >> 8), uint8_t(count)};
}
static float f32(const uint8_t *p) {  // big-endian float32
  uint32_t b = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
  float f; std::memcpy(&f, &b, 4); return f;
}

int main() {
  const float nan = NAN;
  uint8_t out[MAX_RESPONSE_LEN];

  // --- import 2300 W on L1, L2/L3 never reported (NaN), voltage only on L1
  float volt[3] = {230.0f, nan, nan}, pd[3] = {2300.0f, nan, nan}, pr[3] = {0.0f, nan, nan};
  Values v = compute_values(volt, pd, pr, 230.0f);
  assert(std::fabs(v.current[0] - 10.0f) < 1e-4);
  assert(v.power[1] == 0.0f && v.current[1] == 0.0f && v.voltage[1] == 0.0f);  // absent phase = all zero
  assert(v.voltage[0] == 230.0f);

  auto r = req(0x1234, 1, 4, 0, 6);
  size_t n = build_response(r.data(), r.size(), &v, 1, out);
  assert(n == 9 + 12);
  assert(out[0] == 0x12 && out[1] == 0x34 && out[6] == 1 && out[7] == 4 && out[8] == 12);
  assert((out[4] << 8 | out[5]) == 3 + 12);
  assert(f32(out + 9) == 230.0f && f32(out + 13) == 0.0f && f32(out + 17) == 0.0f);

  r = req(1, 1, 4, 6, 6);
  n = build_response(r.data(), r.size(), &v, 1, out);
  assert(n == 21 && std::fabs(f32(out + 9) - 10.0f) < 1e-4 && f32(out + 13) == 0.0f);

  r = req(1, 1, 4, 12, 6);
  n = build_response(r.data(), r.size(), &v, 1, out);
  assert(n == 21 && f32(out + 9) == 2300.0f && f32(out + 13) == 0.0f);

  // --- export 2300 W: signs flip
  float pd2[3] = {0.0f, nan, nan}, pr2[3] = {2300.0f, nan, nan};
  Values ve = compute_values(volt, pd2, pr2, 230.0f);
  r = req(1, 1, 4, 6, 6);
  n = build_response(r.data(), r.size(), &ve, 1, out);
  assert(std::fabs(f32(out + 9) + 10.0f) < 1e-4);
  r = req(1, 1, 4, 12, 6);
  n = build_response(r.data(), r.size(), &ve, 1, out);
  assert(f32(out + 9) == -2300.0f);

  // --- FC3 gives the same data as FC4
  r = req(1, 1, 3, 12, 2);
  n = build_response(r.data(), r.size(), &v, 1, out);
  assert(n == 13 && out[7] == 3 && f32(out + 9) == 2300.0f);

  // --- unaligned read (addr 1, count 2): low half of V1 + high half of V2
  r = req(1, 1, 4, 1, 2);
  n = build_response(r.data(), r.size(), &v, 1, out);
  uint32_t b; float x = 230.0f; std::memcpy(&b, &x, 4);
  assert(out[9] == uint8_t((b & 0xFFFF) >> 8) && out[10] == uint8_t(b & 0xFF));
  assert(out[11] == uint8_t(b >> 24) && out[12] == uint8_t(b >> 16));

  // --- total system power and frequency
  r = req(1, 1, 4, 0x34, 2); n = build_response(r.data(), r.size(), &v, 1, out); assert(f32(out + 9) == 2300.0f);
  r = req(1, 1, 4, 0x46, 2); n = build_response(r.data(), r.size(), &v, 1, out); assert(f32(out + 9) == 50.0f);

  // --- stale data => device failure exception
  r = req(7, 1, 4, 0, 6);
  n = build_response(r.data(), r.size(), nullptr, 1, out);
  assert(n == 9 && out[7] == 0x84 && out[8] == EX_DEVICE_FAILURE && out[0] == 0 && out[1] == 7);

  // --- unsupported function, bad count, bad address, wrong unit
  r = req(1, 1, 6, 0, 1); n = build_response(r.data(), r.size(), &v, 1, out); assert(n == 9 && out[7] == 0x86 && out[8] == 1);
  r = req(1, 1, 4, 0, 0); n = build_response(r.data(), r.size(), &v, 1, out); assert(n == 9 && out[8] == 3);
  r = req(1, 1, 4, 0, 126); n = build_response(r.data(), r.size(), &v, 1, out); assert(n == 9 && out[8] == 3);
  r = req(1, 1, 4, 0x01FF, 2); n = build_response(r.data(), r.size(), &v, 1, out); assert(n == 9 && out[8] == 2);
  r = req(1, 5, 4, 0, 6); n = build_response(r.data(), r.size(), &v, 1, out); assert(n == 0);
  r = req(1, 0xFF, 4, 0, 6); n = build_response(r.data(), r.size(), &v, 1, out); assert(n > 0);

  // --- implausible voltage on a present phase falls back to nominal; absent phases stay 0
  float vlow[3] = {0.0f, nan, nan};
  Values vf = compute_values(vlow, pd, pr, 230.0f);
  assert(vf.voltage[0] == 230.0f && vf.voltage[1] == 0.0f);

  // --- three phases present, no voltage sensors: nominal voltage on all, current = P/U
  float pd3[3] = {690.0f, 230.0f, 0.0f}, pr3[3] = {0.0f, 0.0f, 460.0f}, vn[3] = {nan, nan, nan};
  Values v3 = compute_values(vn, pd3, pr3, 230.0f);
  assert(v3.voltage[2] == 230.0f && std::fabs(v3.current[0] - 3.0f) < 1e-4 && std::fabs(v3.current[1] - 1.0f) < 1e-4 &&
         std::fabs(v3.current[2] + 2.0f) < 1e-4);

  // --- frame_length / TCP reassembly: feed two pipelined requests in 3-byte chunks
  std::vector<uint8_t> stream;
  auto a = req(1, 1, 4, 0, 6), b2 = req(2, 1, 4, 12, 6);
  stream.insert(stream.end(), a.begin(), a.end()); stream.insert(stream.end(), b2.begin(), b2.end());
  uint8_t buf[MAX_FRAME_LEN]; size_t len = 0; int answered = 0;
  for (size_t i = 0; i < stream.size(); i += 3) {
    size_t chunk = std::min<size_t>(3, stream.size() - i);
    std::memcpy(buf + len, stream.data() + i, chunk); len += chunk;
    while (len >= 6) {
      int total = frame_length(buf, len);
      assert(total > 0);
      if (len < size_t(total)) break;
      size_t rl = build_response(buf, total, &v, 1, out);
      assert(rl == 21); answered++;
      std::memmove(buf, buf + total, len - total); len -= total;
    }
  }
  assert(answered == 2 && len == 0);

  uint8_t bad[6] = {0, 1, 0, 5, 0, 6};  // protocol id != 0
  assert(frame_length(bad, 6) == -1);
  uint8_t tiny[6] = {0, 1, 0, 0, 0, 1};  // length < 2
  assert(frame_length(tiny, 6) == -1);
  assert(frame_length(bad, 5) == 0);

  std::printf("ALL PROTOCOL TESTS PASSED\n");
  return 0;
}
