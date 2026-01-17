#pragma once
#include <cstdint>
#include <array>
#include <cstring>

namespace rs {

enum class Comm : uint8_t {
  MotionControl = 0x01,
  MotorRequest  = 0x02,
  MotorEnable   = 0x03,
  MotorStop     = 0x04,
  SetPosZero    = 0x06,
  GetParam      = 0x11,
  SetParam      = 0x12,
  DataSave      = 0x16,
};

struct IdMeta {
  uint8_t comm_type = 0;   // (eid>>24)&0x1F
  uint16_t extra16 = 0;    // (eid>>8)&0xFFFF
  uint8_t low8 = 0;        // eid&0xFF

  uint8_t motor_id_bits15_8 = 0; // (eid>>8)&0xFF
  uint8_t host_id_bits7_0   = 0; // eid&0xFF

  uint8_t error_code = 0;  // (eid>>16)&0x3F
  uint8_t pattern    = 0;  // (eid>>22)&0x03
};

inline uint32_t make_eid(uint8_t type5, uint16_t extra16, uint8_t low8) {
  return ((uint32_t)(type5 & 0x1F) << 24) | ((uint32_t)extra16 << 8) | (uint32_t)low8;
}

inline IdMeta decode_eid(uint32_t eid) {
  IdMeta m;
  m.comm_type = (eid >> 24) & 0x1F;
  m.extra16   = (eid >> 8)  & 0xFFFF;
  m.low8      = eid & 0xFF;

  m.motor_id_bits15_8 = (eid >> 8) & 0xFF;
  m.host_id_bits7_0   = eid & 0xFF;

  m.error_code = (eid >> 16) & 0x3F;
  m.pattern    = (eid >> 22) & 0x03;
  return m;
}

inline uint16_t float_to_u16(float x, float xmin, float xmax) {
  if (x < xmin) x = xmin;
  if (x > xmax) x = xmax;
  float span = xmax - xmin;
  float off  = x - xmin;
  return (uint16_t)((off * 65535.0f) / span + 0.5f);
}

inline void be16_store(std::array<uint8_t,8>& d, int idx, uint16_t v) {
  d[idx]   = (uint8_t)(v >> 8);
  d[idx+1] = (uint8_t)(v & 0xFF);
}

inline float pvtt_decode_u16(uint16_t u, float range_plus_minus) {
  return ((2.0f * float(u) / 65535.0f) - 1.0f) * range_plus_minus;
}

// 参数读写是 little-endian（和 PVTT 的 be16 不同）
inline uint16_t le16_load(const uint8_t* d, int idx) {
  return (uint16_t)d[idx] | ((uint16_t)d[idx + 1] << 8);
}

inline uint32_t le32_load(const uint8_t* d, int idx) {
  return (uint32_t)d[idx] |
         ((uint32_t)d[idx + 1] << 8) |
         ((uint32_t)d[idx + 2] << 16) |
         ((uint32_t)d[idx + 3] << 24);
}

} // namespace rs
