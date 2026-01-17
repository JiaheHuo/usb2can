#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <optional>
#include <array>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <Eigen/Core>

// ====== 启动校准开关：允许 CMake -D 覆盖 ======
#ifndef IMU_CALIB_ACCEL_ON_START
#define IMU_CALIB_ACCEL_ON_START 0    // 1→每次启动做加速度校准
#endif

#ifndef IMU_CALIB_ANGLEREF_ON_START
#define IMU_CALIB_ANGLEREF_ON_START 0 // 1→XY 角度参考归零
#endif

#ifndef IMU_CALIB_Z_ON_START
#define IMU_CALIB_Z_ON_START 0       // 1→Z 轴置零（六轴算法模式）
#endif

// ==========================
//  IMU 输出状态（欧拉角）
// ==========================
struct ImuState {
  bool has_accel = false;
  bool has_gyro  = false;
  bool has_euler = false; // 来自 0x53

  Eigen::Vector3d accel_mps2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_rps   = Eigen::Vector3d::Zero();

  // WitMotion 0x53：roll/pitch/yaw（单位：deg）
  Eigen::Vector3d rpy_deg    = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy_rad    = Eigen::Vector3d::Zero();

  uint64_t seq = 0;
};

// ==========================
//  串口（termios）
// ==========================
class SerialPort {
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  bool open(const std::string& port, int baud, int vmin, int vtime);
  bool open(const std::string& port, int baud) { return open(port, baud, 1, 0); }
  void close();
  bool is_open() const;

  int  read_some(uint8_t* buf, std::size_t max_len);          // 阻塞读
  bool write_all(const uint8_t* data, std::size_t len);       // 写满

private:
  int fd_ = -1;
};

// ==========================
//  WitMotion：解析 0x51/0x52/0x53 + 校准命令
// ==========================
class WitMotionImu {
public:
  WitMotionImu() = default;

  // 喂入串口字节流；若解析到有效帧并更新任一字段，返回增量状态
  std::optional<ImuState> feed(const uint8_t* data, std::size_t n);

  // 校准/置零命令（沿用你之前那套 0xFF 0xAA ...）
  void calib_accel(SerialPort& sp);
  void calib_angle_ref(SerialPort& sp);
  void calib_z_axis(SerialPort& sp);

private:
  // frame: 0x55 + type + payload(8) + sum(1) = 11 bytes
  std::array<uint8_t, 11> frame_{};
  int frame_pos_ = 0;

  ImuState st_{};
  uint64_t seq_ = 0;

private:
  static uint8_t checksum10(const uint8_t* p);

  void send_unlock_(SerialPort& sp);
  void save_change_(SerialPort& sp);

  void handle_frame_(const std::array<uint8_t,11>& f, ImuState& out);
};
class WitMotionImuRunner {
public:
  struct Options {
    std::string port = "/dev/imu";
    int baud = 921600;          // 你要求默认 921600
    bool calib_accel_on_start = (IMU_CALIB_ACCEL_ON_START != 0);
    bool calib_angleref_on_start = (IMU_CALIB_ANGLEREF_ON_START != 0);
    bool calib_z_on_start = (IMU_CALIB_Z_ON_START != 0);
  };

  WitMotionImuRunner() = default;
  ~WitMotionImuRunner() { stop(); }

  bool start(const Options& opt);
  void stop();

  // 拿最新样本 + age_ms；没有数据则返回 false
  bool get_latest(ImuState& out, uint32_t* age_ms = nullptr) const;

  bool running() const { return running_.load(); }

private:
  void thread_fn_(Options opt);

private:
  mutable std::mutex mtx_;
  ImuState last_{};
  bool has_any_ = false;
  std::chrono::steady_clock::time_point last_rx_tp_{};

  std::atomic<bool> running_{false};
  std::atomic<bool> stop_req_{false};
  std::thread th_;
};