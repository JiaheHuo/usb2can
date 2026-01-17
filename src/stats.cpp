#include "stats.hpp"
#include <iostream>
#include <iomanip>
#include <map>
#include <sstream>
// 格式：YYYY-MM-DD HH:MM:SS.mmm（本地时间）
static std::string wall_now_string() {
  using namespace std::chrono;
  auto tp = system_clock::now();
  auto t  = system_clock::to_time_t(tp);

  std::tm tm{};
  localtime_r(&t, &tm);  // 线程安全

  auto ms = duration_cast<milliseconds>(tp.time_since_epoch()) % 1000;

  std::ostringstream oss;
  oss << std::put_time(&tm, "%F %T")
      << "." << std::setw(3) << std::setfill('0') << ms.count();
  return oss.str();
}
int Stats::register_motor(const MotorKey& k) {
  motors_.emplace_back();        // 原地构造，不触发 move
  motors_.back().key = k;
  return (int)motors_.size() - 1;
}

void Stats::inc_tx_motion(int idx){ motors_[idx].cnt.tx_motion.fetch_add(1, std::memory_order_relaxed); }
void Stats::inc_tx_other (int idx){ motors_[idx].cnt.tx_other .fetch_add(1, std::memory_order_relaxed); }
void Stats::inc_rx_status(int idx){ motors_[idx].cnt.rx_status.fetch_add(1, std::memory_order_relaxed); }
void Stats::inc_rx_other (int idx){ motors_[idx].cnt.rx_other .fetch_add(1, std::memory_order_relaxed); }

void Stats::print_periodic() {
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_print_).count();
  double elapsed = std::chrono::duration<double>(now - start_).count();
  if (dt < 1.0) return;
  last_print_ = now;

  struct Agg {
    uint64_t tx_attempt=0, tx_ok=0, tx_fail=0;
    uint64_t tx_other=0, rx_status=0, rx_other=0;
  };

  std::map<std::string, Agg> devAgg;
  std::map<std::string, Agg> busAgg;

  Agg total{};

  for (auto& e : motors_) {
    const uint64_t tx_attempt = e.cnt.tx_motion.load(std::memory_order_relaxed);
    const uint64_t tx_ok      = e.cnt.tx_motion_ok.load(std::memory_order_relaxed);
    const uint64_t tx_fail    = e.cnt.tx_motion_fail.load(std::memory_order_relaxed);
    const uint64_t tx_other   = e.cnt.tx_other.load(std::memory_order_relaxed);
    const uint64_t rx_status  = e.cnt.rx_status.load(std::memory_order_relaxed);
    const uint64_t rx_other   = e.cnt.rx_other.load(std::memory_order_relaxed);

    const uint64_t d_tx_attempt = tx_attempt - e.last_tx_motion;
    const uint64_t d_tx_ok      = tx_ok      - e.last_tx_motion_ok;
    const uint64_t d_tx_fail    = tx_fail    - e.last_tx_motion_fail;
    const uint64_t d_tx_other   = tx_other   - e.last_tx_other;
    const uint64_t d_rx_status  = rx_status  - e.last_rx_status;
    const uint64_t d_rx_other   = rx_other   - e.last_rx_other;

    e.last_tx_motion      = tx_attempt;
    e.last_tx_motion_ok   = tx_ok;
    e.last_tx_motion_fail = tx_fail;
    e.last_tx_other       = tx_other;
    e.last_rx_status      = rx_status;
    e.last_rx_other       = rx_other;

    // total
    total.tx_attempt += d_tx_attempt;
    total.tx_ok      += d_tx_ok;
    total.tx_fail    += d_tx_fail;
    total.tx_other   += d_tx_other;
    total.rx_status  += d_rx_status;
    total.rx_other   += d_rx_other;

    // key
    devAgg[e.key.dev_name].tx_attempt += d_tx_attempt;
    devAgg[e.key.dev_name].tx_ok      += d_tx_ok;
    devAgg[e.key.dev_name].tx_fail    += d_tx_fail;
    devAgg[e.key.dev_name].tx_other   += d_tx_other;
    devAgg[e.key.dev_name].rx_status  += d_rx_status;
    devAgg[e.key.dev_name].rx_other   += d_rx_other;

    std::string bkey = e.key.dev_name + ":bus" + std::to_string((int)e.key.channel);
    busAgg[bkey].tx_attempt += d_tx_attempt;
    busAgg[bkey].tx_ok      += d_tx_ok;
    busAgg[bkey].tx_fail    += d_tx_fail;
    busAgg[bkey].tx_other   += d_tx_other;
    busAgg[bkey].rx_status  += d_rx_status;
    busAgg[bkey].rx_other   += d_rx_other;
  }

  auto rate = [&](uint64_t n){ return double(n)/dt; };
  auto pct  = [&](uint64_t a, uint64_t b){ return (b>0) ? (100.0*(1.0 - double(a)/double(b))) : 0.0; };

  // 旧 drop：用 attempt 当分母（会把“没发出去”也算成丢）
  const double drop_est_attempt = pct(total.rx_status, total.tx_attempt);

  // 真 drop：用 tx_ok 当分母（只统计“确实发出去”的帧）
  const double drop_true = pct(total.rx_status, total.tx_ok);

  const double tx_fail_rate = (total.tx_ok + total.tx_fail > 0)
      ? (100.0 * double(total.tx_fail) / double(total.tx_ok + total.tx_fail))
      : 0.0;

  std::cout << "\n[" << wall_now_string() << "]"
            << " [t=" << std::fixed << std::setprecision(3) << elapsed << "s]"
            << "\n[" << std::fixed << std::setprecision(3) << dt << "s] "
            << "TX_attempt=" << total.tx_attempt << " (" << rate(total.tx_attempt) << " Hz)  "
            << "TX_ok=" << total.tx_ok << " (" << rate(total.tx_ok) << " Hz)  "
            << "TX_fail=" << total.tx_fail << " (" << tx_fail_rate << "%)  "
            << "RX_status=" << total.rx_status << " (" << rate(total.rx_status) << " Hz)\n"
            << "drop_est(attempt)~=" << drop_est_attempt << "%  "
            << "drop_true(tx_ok)~=" << drop_true << "%  "
            << "RX_other=" << total.rx_other << " (" << rate(total.rx_other) << " Hz)\n";

  for (auto& [k, a] : devAgg) {
    double dtrue = pct(a.rx_status, a.tx_ok);
    double tfail = (a.tx_ok + a.tx_fail > 0) ? (100.0 * double(a.tx_fail)/double(a.tx_ok+a.tx_fail)) : 0.0;
    std::cout << "  " << k
              << ": TX_ok=" << a.tx_ok << " TX_fail=" << a.tx_fail << " (" << tfail << "%)"
              << " RX_status=" << a.rx_status
              << " drop_true~=" << dtrue << "%\n";
  }

  std::cout << "======================================\n";
  for (auto& [k, a] : busAgg) {
    double dtrue = pct(a.rx_status, a.tx_ok);
    double tfail = (a.tx_ok + a.tx_fail > 0) ? (100.0 * double(a.tx_fail)/double(a.tx_ok+a.tx_fail)) : 0.0;
    std::cout << "    " << k
              << ": TX_ok=" << a.tx_ok << " TX_fail=" << a.tx_fail << " (" << tfail << "%)"
              << " RX_status=" << a.rx_status
              << " drop_true~=" << dtrue << "%\n";
  }

  if (total.tx_ok + total.tx_fail != total.tx_attempt) {
    std::cout << "!! WARN: TX_ok+TX_fail != TX_attempt ("
              << (total.tx_ok + total.tx_fail) << " vs " << total.tx_attempt
              << "). Check motor.cpp counters.\n";
  }
}
