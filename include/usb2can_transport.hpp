#pragma once
#include <string>
#include <array>
#include <stdexcept>
#include <cstdint>
#include <mutex>
#include <chrono>
#include <thread>

#include "usb_can.h"

struct Usb2CanFrame {
  uint8_t channel = 0; // 1 or 2
  uint32_t eid = 0;
  std::array<uint8_t,8> data{};
  uint8_t dlc = 8;
};

class Usb2CanDevice {
public:
  Usb2CanDevice() = default;

  explicit Usb2CanDevice(const std::string& dev_path, int tx_delay_us = 75)
  : tx_delay_us_(tx_delay_us)
  {
    open(dev_path);
  }
  void open(const std::string& dev_path) {
    path_ = dev_path;
    fd_ = openUSBCAN(dev_path.c_str());
    if (fd_ < 0) throw std::runtime_error("openUSBCAN failed: " + dev_path);

    // 初始化 last_tx_tp_，让第一次 send() 不被“补偿睡眠”影响
    auto now = std::chrono::steady_clock::now();
    for (auto& tp : last_tx_tp_) tp = now;

    next_tx_tp_ = std::chrono::steady_clock::now();
  }

  ~Usb2CanDevice() {
    if (fd_ >= 0) closeUSBCAN(fd_);
  }

  const std::string& path() const { return path_; }
  int fd() const { return fd_; }

  void set_tx_delay_us(int us) { tx_delay_us_ = us; }

  bool send(uint8_t channel, uint32_t eid, const std::array<uint8_t,8>& data, uint8_t dlc=8) 
  {
    if (fd_ < 0) throw std::runtime_error("Usb2CanDevice::send called but device not opened: " + path_);
    if (channel < 1 || channel > 2) throw std::runtime_error("Usb2CanDevice::send invalid channel (expect 1/2)");

    // ✅ 每个 device 的每个 channel 独立节流
    if (tx_delay_us_ > 0) {
      std::lock_guard<std::mutex> lk(tx_mu_[channel]);

      auto now = std::chrono::steady_clock::now();
      auto last = last_tx_tp_[channel];
      auto used_us = std::chrono::duration_cast<std::chrono::microseconds>(now - last).count();

      if (used_us < tx_delay_us_) {
        std::this_thread::sleep_for(std::chrono::microseconds(tx_delay_us_ - used_us));
      }
      last_tx_tp_[channel] = std::chrono::steady_clock::now();
    }

    FrameInfo info{};
    info.canID = eid;
    info.frameType = EXTENDED;
    info.dataLength = dlc;

    uint8_t raw[8];
    for (int i = 0; i < 8; i++)
      raw[i] = data[i];

    if (tx_delay_us_ > 0)
    {
      std::lock_guard<std::mutex> lk(tx_mu_global_);
      auto now = std::chrono::steady_clock::now();
      if (now < next_tx_tp_)
        std::this_thread::sleep_until(next_tx_tp_);
      next_tx_tp_ = std::chrono::steady_clock::now() + std::chrono::microseconds(tx_delay_us_);
    }

    int ret = sendUSBCAN(fd_, channel, &info, raw);
    return true;
  }

  bool recv(Usb2CanFrame& out, int timeout_us) {
    if (fd_ < 0) return false;

    FrameInfo info_rx{};
    uint8_t channel = 0;
    uint8_t raw[8] = {0};

    int ret = readUSBCAN(fd_, &channel, &info_rx, raw, timeout_us);
    if (ret == -1) return false; // timeout 或无数据（按 Tangair 的 API 约定）

    out.channel = channel;
    out.eid = info_rx.canID;
    out.dlc = info_rx.dataLength;
    for (int i=0;i<8;i++) out.data[i] = raw[i];
    return true;
  }

private:
  std::string path_;
  int fd_ = -1;

  // ✅ 发送节流：每 channel 独立
  int tx_delay_us_{75};
  std::array<std::mutex, 3> tx_mu_;
  std::array<std::chrono::steady_clock::time_point, 3> last_tx_tp_;
  std::mutex tx_mu_global_;
  std::chrono::steady_clock::time_point next_tx_tp_{};
};
