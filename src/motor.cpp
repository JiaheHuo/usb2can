#include "motor.hpp"
#include <cstring>

static constexpr uint16_t RS_PARAM_ZERO_STA = 0x7029; //

RobStrideMotor::RobStrideMotor(BusHandle bus, uint8_t master_id, uint8_t motor_id, ActuatorType type,
                               std::string group, Stats* stats, int stats_idx, std::string motorName)
: bus_(bus), master_id_(master_id), motor_id_(motor_id), type_(type),
  group_(std::move(group)), stats_(stats), stats_idx_(stats_idx),motorName_(motorName) {}

void RobStrideMotor::enable() {
  std::array<uint8_t,8> data{}; data.fill(0);
  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::MotorEnable, master_id_, motor_id_);
  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

void RobStrideMotor::stop(uint8_t clear_error) {
  std::array<uint8_t,8> data{}; data.fill(0);
  data[0] = clear_error;
  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::MotorStop, master_id_, motor_id_);
  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

void RobStrideMotor::set_zero() {
  std::array<uint8_t,8> data{}; data.fill(0);
  data[0] = 1;
  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::SetPosZero, master_id_, motor_id_);
  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

void RobStrideMotor::set_param_f32(uint16_t index, float value) {
  std::array<uint8_t,8> data{}; data.fill(0);
  data[0] = (uint8_t)(index & 0xFF);
  data[1] = (uint8_t)(index >> 8);
  std::memcpy(&data[4], &value, 4);

  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::SetParam, master_id_, motor_id_);
  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

void RobStrideMotor::set_param_u8(uint16_t index, uint8_t value) {
  std::array<uint8_t,8> data{}; data.fill(0);
  data[0] = (uint8_t)(index & 0xFF);
  data[1] = (uint8_t)(index >> 8);
  data[4] = value;

  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::SetParam, master_id_, motor_id_);
  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

void RobStrideMotor::get_param(uint16_t index) {
  std::array<uint8_t,8> data{}; data.fill(0);
  data[0] = (uint8_t)(index & 0xFF);
  data[1] = (uint8_t)(index >> 8);

  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::GetParam, master_id_, motor_id_);
  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

void RobStrideMotor::set_mode(uint8_t run_mode) {
  // 0x7005: run_mode
  set_param_u8(0x7005, run_mode);
}

void RobStrideMotor::send_motion_command(float torque, float pos_rad, float vel_rad_s, float kp, float kd) {
  const auto op = actuator_map().at(type_);

  uint16_t t_u = rs::float_to_u16(torque, -op.tau, op.tau);
  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::MotionControl, t_u, motor_id_);

  std::array<uint8_t,8> data{}; data.fill(0);
  uint16_t p_u  = rs::float_to_u16(pos_rad, -op.pos, op.pos);
  uint16_t v_u  = rs::float_to_u16(vel_rad_s, -op.vel, op.vel);
  uint16_t kp_u = rs::float_to_u16(kp, 0.f, op.kp);
  uint16_t kd_u = rs::float_to_u16(kd, 0.f, op.kd);

  rs::be16_store(data, 0, p_u);
  rs::be16_store(data, 2, v_u);
  rs::be16_store(data, 4, kp_u);
  rs::be16_store(data, 6, kd_u);

  bool ok = bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_)
  {
    stats_->inc_tx_motion(stats_idx_); // attempt
    if (ok)
      stats_->inc_tx_motion_ok(stats_idx_);
    else
      stats_->inc_tx_motion_fail(stats_idx_);
  }
}

void RobStrideMotor::on_frame(const Usb2CanFrame& f) {
  auto meta = rs::decode_eid(f.eid);

  if (meta.comm_type == (uint8_t)rs::Comm::MotorRequest) {
    uint16_t pu = (f.data[0] << 8) | f.data[1];
    uint16_t vu = (f.data[2] << 8) | f.data[3];
    uint16_t tu = (f.data[4] << 8) | f.data[5];
    uint16_t te = (f.data[6] << 8) | f.data[7];

    const auto op = actuator_map().at(type_);

    PVTT pv;
    pv.p    = rs::pvtt_decode_u16(pu, op.pos);
    pv.v    = rs::pvtt_decode_u16(vu, op.vel);
    pv.t    = rs::pvtt_decode_u16(tu, op.tau);
    pv.temp = float(te) * 0.1f;

    {
      std::lock_guard<std::mutex> lk(mu_);
      pvtt_ = pv;
      has_pvtt_ = true;
    }
    if (stats_) stats_->inc_rx_status(stats_idx_);
    return;
  }
  if(meta.comm_type == (uint8_t)rs::Comm::GetParam) {
    // Byte0~1 index: little-endian
    uint16_t idx = rs::le16_load(f.data.data(), 0);
    // Byte4~7 value: little-endian
    uint32_t val = rs::le32_load(f.data.data(), 4);
    {
      std::lock_guard<std::mutex> lk(mu_);
      last_param_index_ = idx;
      last_param_u32_   = val;
      last_param_valid_ = true;
      last_param_seq_++;
    }
    cv_param_.notify_all();
    if (stats_) stats_->inc_rx_other(stats_idx_);
    return;
  }
  if(stats_) stats_->inc_rx_other(stats_idx_);
}

std::optional<PVTT> RobStrideMotor::last_pvtt() const {
  std::lock_guard<std::mutex> lk(mu_);
  if (!has_pvtt_) return std::nullopt;
  return pvtt_;
}

void RobStrideMotor::data_save() {
  std::array<uint8_t,8> data{}; // 通信类型22 电机数据保存帧 8Byte 数据区

  data = {1,2,3,4,5,6,7,8};
  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::DataSave, master_id_, motor_id_);

  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

std::optional<uint32_t> RobStrideMotor::get_param_u32_blocking(uint16_t index, int timeout_ms) {
  uint32_t start_seq = 0;
  {
    std::lock_guard<std::mutex> lk(mu_);
    start_seq = last_param_seq_;
  }

  get_param(index);

  std::unique_lock<std::mutex> lk(mu_);
  bool ok = cv_param_.wait_for(
      lk, std::chrono::milliseconds(timeout_ms),
      [&]{
        return last_param_valid_ &&
               last_param_index_ == index &&
               last_param_seq_ != start_seq;
      });

  if (!ok) return std::nullopt;
  return last_param_u32_;
}

std::optional<float> RobStrideMotor::get_param_f32_blocking(uint16_t index, int timeout_ms) {
  auto u = get_param_u32_blocking(index, timeout_ms);
  if (!u) return std::nullopt;
  float f = 0.f;
  std::memcpy(&f, &(*u), 4);
  return f;
}

std::optional<uint8_t> RobStrideMotor::get_zero_sta(int timeout_ms) {
  auto u = get_param_u32_blocking(RS_PARAM_ZERO_STA, timeout_ms);
  if (!u) return std::nullopt;
  return (uint8_t)(*u & 0xFF);
}

bool RobStrideMotor::set_zero_sta(uint8_t v, bool persist, int timeout_ms) {
  // v: 0 -> 0~2π, 1 -> -π~π
  set_param_u8(RS_PARAM_ZERO_STA, v);

  if (persist) {
    data_save(); // 掉电保持
  }

  auto rb = get_zero_sta(timeout_ms);
  return rb && (*rb == v);
}