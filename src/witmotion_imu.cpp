#include "witmotion_imu.hpp"

#include <iostream>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iomanip>

// ==========================
//  SerialPort 实现
// ==========================
static speed_t baud_to_termios(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return 0;
  }
}

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& port, int baud, int vmin, int vtime) {
  close();

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    std::cerr << "[Serial] open failed: " << port
              << " err=" << std::strerror(errno) << "\n";
    return false;
  }

  termios tio{};
  tio.c_cc[VTIME] = (cc_t)std::max(0, std::min(255, vtime));
  tio.c_cc[VMIN]  = (cc_t)std::max(0, std::min(255, vmin));
  if (tcgetattr(fd_, &tio) != 0) {
    std::cerr << "[Serial] tcgetattr failed: " << std::strerror(errno) << "\n";
    close();
    return false;
  }

  // raw 模式
  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;

  // 阻塞：至少读到 1 字节返回
  tio.c_cc[VTIME] = 0;
  tio.c_cc[VMIN]  = 1;

  speed_t spd = baud_to_termios(baud);
  if (spd == 0) {
    std::cerr << "[Serial] unsupported baud: " << baud << "\n";
    close();
    return false;
  }

  if (cfsetispeed(&tio, spd) != 0 || cfsetospeed(&tio, spd) != 0) {
    std::cerr << "[Serial] set baud failed: " << std::strerror(errno) << "\n";
    close();
    return false;
  }

  tcflush(fd_, TCIFLUSH);
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    std::cerr << "[Serial] tcsetattr failed: " << std::strerror(errno) << "\n";
    close();
    return false;
  }

  std::cout << "[Serial] opened " << port << " @ " << baud << "\n";
  return true;
}

void SerialPort::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const { return fd_ >= 0; }

int SerialPort::read_some(uint8_t* buf, std::size_t max_len) {
  if (fd_ < 0) return -1;
  int n = ::read(fd_, buf, max_len);
  if (n < 0) {
    if (errno == EINTR) return 0;
    std::cerr << "[Serial] read failed: " << std::strerror(errno) << "\n";
    return -1;
  }
  return n;
}

bool SerialPort::write_all(const uint8_t* data, std::size_t len) {
  if (fd_ < 0) return false;
  std::size_t off = 0;
  while (off < len) {
    ssize_t n = ::write(fd_, data + off, len - off);
    if (n < 0) {
      if (errno == EINTR) continue;
      std::cerr << "[Serial] write failed: " << std::strerror(errno) << "\n";
      return false;
    }
    off += static_cast<std::size_t>(n);
  }
  return true;
}

// ==========================
//  WitMotionImu 实现（重点：0x53 欧拉角）
// ==========================
static inline void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

uint8_t WitMotionImu::checksum10(const uint8_t* p) {
  uint8_t s = 0;
  for (int i = 0; i < 10; ++i) s = static_cast<uint8_t>(s + p[i]);
  return s;
}

void WitMotionImu::send_unlock_(SerialPort& sp) {
  const uint8_t cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
  sp.write_all(cmd, 5);
  sleep_ms(200);
}

void WitMotionImu::save_change_(SerialPort& sp) {
  const uint8_t cmd[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
  sp.write_all(cmd, 5);
}

void WitMotionImu::calib_accel(SerialPort& sp) {
  send_unlock_(sp);
  const uint8_t start[5] = {0xFF, 0xAA, 0x01, 0x01, 0x00};
  const uint8_t exitC[5] = {0xFF, 0xAA, 0x01, 0x00, 0x00};
  sp.write_all(start, 5);
  std::this_thread::sleep_for(std::chrono::seconds(4));
  sp.write_all(exitC, 5);
  sleep_ms(100);
  save_change_(sp);
}

void WitMotionImu::calib_angle_ref(SerialPort& sp) {
  send_unlock_(sp);
  const uint8_t cmd[5] = {0xFF, 0xAA, 0x01, 0x08, 0x00};
  sp.write_all(cmd, 5);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  save_change_(sp);
}

void WitMotionImu::calib_z_axis(SerialPort& sp) {
  send_unlock_(sp);
  const uint8_t cmd[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
  sp.write_all(cmd, 5);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  save_change_(sp);
}

void WitMotionImu::handle_frame_(const std::array<uint8_t,11>& f, ImuState& out) {
  const uint8_t type = f[1];
  const uint8_t* payload = &f[2];

  auto read_i16 = [&](int idx)->int16_t {
    int16_t v;
    std::memcpy(&v, payload + idx*2, sizeof(int16_t));
    return v;
  };

  if (type == 0x51) { // accel
    int16_t ax = read_i16(0);
    int16_t ay = read_i16(1);
    int16_t az = read_i16(2);

    const double g = 9.80665;
    out.accel_mps2.x() = (double)ax / 32768.0 * 16.0 * g;
    out.accel_mps2.y() = (double)ay / 32768.0 * 16.0 * g;
    out.accel_mps2.z() = (double)az / 32768.0 * 16.0 * g;
    out.has_accel = true;
  }
  else if (type == 0x52) { // gyro
    int16_t gx = read_i16(0);
    int16_t gy = read_i16(1);
    int16_t gz = read_i16(2);

    const double deg2rad = M_PI / 180.0;
    out.gyro_rps.x() = (double)gx / 32768.0 * 2000.0 * deg2rad;
    out.gyro_rps.y() = (double)gy / 32768.0 * 2000.0 * deg2rad;
    out.gyro_rps.z() = (double)gz / 32768.0 * 2000.0 * deg2rad;
    out.has_gyro = true;
  }
  else if (type == 0x53) { // Euler angles (roll/pitch/yaw) —— 你要的“直接读欧拉角”
    int16_t r = read_i16(0);
    int16_t p = read_i16(1);
    int16_t y = read_i16(2);

    // 官方写法等价：angle = sAngle/32768*180 (deg)
    const double roll_deg  = (double)r / 32768.0 * 180.0;
    const double pitch_deg = (double)p / 32768.0 * 180.0;
    const double yaw_deg   = (double)y / 32768.0 * 180.0;

    out.rpy_deg = Eigen::Vector3d(roll_deg, pitch_deg, yaw_deg);
    out.rpy_rad = out.rpy_deg * (M_PI / 180.0);
    out.has_euler = true;
  }
}

std::optional<ImuState> WitMotionImu::feed(const uint8_t* data, std::size_t n) {
  std::optional<ImuState> out;

  for (std::size_t k = 0; k < n; ++k) {
    const uint8_t b = data[k];

    if (frame_pos_ == 0) {
      if (b == 0x55) {
        frame_[0] = 0x55;
        frame_pos_ = 1;
      }
      continue;
    }

    frame_[frame_pos_++] = b;

    if (frame_pos_ == 11) {
      frame_pos_ = 0;

      if (checksum10(frame_.data()) != frame_[10]) {
        continue;
      }

      ImuState sample = st_; // 增量更新
      sample.has_accel = false;
      sample.has_gyro  = false;
      sample.has_euler = false;

      handle_frame_(frame_, sample);
      st_ = sample;

      if (sample.has_accel || sample.has_gyro || sample.has_euler) {
        sample.seq = ++seq_;
        out = sample;
      }
    }
  }

  return out;
}

// ==========================
//  main：读取并打印（优先打印欧拉角）
// ==========================
static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run.store(false); }

struct Args {
  std::string port = "/dev/imu";
  int baud = 115200;   // 你这份官方角度版本默认 115200
  int print_hz = 50;   // 控制打印频率
};

static Args parse_args(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    auto next = [&](const char* key)->const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << key << "\n";
        std::exit(1);
      }
      return argv[++i];
    };

    if (std::strcmp(argv[i], "--port") == 0) a.port = next("--port");
    else if (std::strcmp(argv[i], "--baud") == 0) a.baud = std::stoi(next("--baud"));
    else if (std::strcmp(argv[i], "--print_hz") == 0) a.print_hz = std::stoi(next("--print_hz"));
    else if (std::strcmp(argv[i], "--help") == 0) {
      std::cout << "Usage: witmotion_imu --port /dev/imu --baud 921600 --print_hz 50\n";
      std::exit(0);
    } else {
      std::cerr << "Unknown arg: " << argv[i] << " (use --help)\n";
      std::exit(1);
    }
  }
  return a;
}

// int main(int argc, char** argv) {
//   std::signal(SIGINT, on_sigint);
//   Args args = parse_args(argc, argv);

//   SerialPort sp;
//   if (!sp.open(args.port, args.baud)) return 1;

//   WitMotionImu imu;

// #if IMU_CALIB_ACCEL_ON_START
//   std::cout << "[IMU] Calib Accel...\n";
//   imu.calib_accel(sp);
//   std::cout << "[IMU] Calib Accel Done.\n";
// #endif
// #if IMU_CALIB_ANGLEREF_ON_START
//   std::cout << "[IMU] XY AngleRef Zeroing...\n";
//   imu.calib_angle_ref(sp);
//   std::cout << "[IMU] XY AngleRef Zeroed.\n";
// #endif
// #if IMU_CALIB_Z_ON_START
//   std::cout << "[IMU] Z Axis Zeroing...\n";
//   imu.calib_z_axis(sp);
//   std::cout << "[IMU] Z Axis Zeroed.\n";
// #endif

//   std::cout << "[IMU] Reading... (Ctrl+C to exit)\n";

//   using clock = std::chrono::steady_clock;
//   auto last_print = clock::now();
//   const auto print_period = (args.print_hz > 0)
//     ? std::chrono::microseconds(1000000 / args.print_hz)
//     : std::chrono::microseconds(0);

//   uint8_t buf[512];

//   ImuState last{};
//   bool has_any = false;

//   while (g_run.load()) {
//     int n = sp.read_some(buf, sizeof(buf));
//     if (n < 0) break;
//     if (n == 0) continue;

//     auto s = imu.feed(buf, static_cast<std::size_t>(n));
//     if (s) {
//       if (s->has_accel) last.accel_mps2 = s->accel_mps2;
//       if (s->has_gyro)  last.gyro_rps   = s->gyro_rps;
//       if (s->has_euler) { last.rpy_deg = s->rpy_deg; last.rpy_rad = s->rpy_rad; }
//       last.seq = s->seq;
//       has_any = true;
//     }

//     if (args.print_hz > 0 && has_any)
//     {
//         auto p3 = [&](double v)
//         {
//             std::cout << std::setw(8) << std::fixed << std::setprecision(3) << v;
//         };
//         auto now = clock::now();
//         if (now - last_print >= print_period)
//         {
//             last_print = now;
//             std::cout << "seq=" << std::setw(6) << last.seq << "  acc ";
//             p3(last.accel_mps2.x());
//             std::cout << " ";
//             p3(last.accel_mps2.y());
//             std::cout << " ";
//             p3(last.accel_mps2.z());

//             std::cout << "  rpy ";
//             std::cout << std::setw(7) << std::fixed << std::setprecision(2) << last.rpy_deg.x() << " ";
//             std::cout << std::setw(7) << std::fixed << std::setprecision(2) << last.rpy_deg.y() << " ";
//             std::cout << std::setw(7) << std::fixed << std::setprecision(2) << last.rpy_deg.z() << "\n";
//         }
//     }
//   }

//   sp.close();
//   std::cout << "[IMU] Exit.\n";
//   return 0;
// }
bool WitMotionImuRunner::start(const Options& opt) {
  stop();
  stop_req_.store(false);
  running_.store(true);
  th_ = std::thread(&WitMotionImuRunner::thread_fn_, this, opt);
  return true;
}

void WitMotionImuRunner::stop() {
  stop_req_.store(true);
  if (th_.joinable()) th_.join();
  running_.store(false);
}

bool WitMotionImuRunner::get_latest(ImuState& out, uint32_t* age_ms) const {
  std::lock_guard<std::mutex> lk(mtx_);
  if (!has_any_) return false;
  out = last_;
  if (age_ms) {
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx_tp_).count();
    *age_ms = (uint32_t)std::max<int64_t>(0, ms);
  }
  return true;
}

void WitMotionImuRunner::thread_fn_(Options opt) {
  SerialPort sp;
  // 关键：VMIN=0, VTIME=1 -> read 最多阻塞 0.1s，便于 stop
  if (!sp.open(opt.port, opt.baud, /*vmin=*/0, /*vtime=*/1)) {
    running_.store(false);
    return;
  }

  WitMotionImu imu;

  if (opt.calib_accel_on_start) imu.calib_accel(sp);
  if (opt.calib_angleref_on_start) imu.calib_angle_ref(sp);
  if (opt.calib_z_on_start) imu.calib_z_axis(sp);

  uint8_t buf[512];

  while (!stop_req_.load()) {
    int n = sp.read_some(buf, sizeof(buf));
    if (n < 0) break;
    if (n == 0) continue;

    auto s = imu.feed(buf, (std::size_t)n);
    if (!s) continue;

    std::lock_guard<std::mutex> lk(mtx_);
    // 增量合并
    if (s->has_accel) last_.accel_mps2 = s->accel_mps2;
    if (s->has_gyro)  last_.gyro_rps   = s->gyro_rps;
    if (s->has_euler) { last_.rpy_deg = s->rpy_deg; last_.rpy_rad = s->rpy_rad; }
    last_.seq = s->seq;
    has_any_ = true;
    last_rx_tp_ = std::chrono::steady_clock::now();
  }

  sp.close();
  running_.store(false);
}
