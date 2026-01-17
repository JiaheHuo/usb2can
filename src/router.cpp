#include "router.hpp"

Usb2CanRouter::Usb2CanRouter(const std::vector<Usb2CanDevice*>& devs)
: devs_(devs)
{
  table_.resize(devs_.size());
  sync_.resize(devs_.size());

  for (size_t di=0; di<devs_.size(); ++di) {
    for (int ch=0; ch<3; ++ch) {
      for (int mid=0; mid<256; ++mid) table_[di][ch][mid] = nullptr;
    }
    for (int ch=1; ch<=2; ++ch) {
      sync_[di].bus[ch].expected.reset();
      sync_[di].bus[ch].reset_round();
      sync_[di].bus[ch].expected_cnt = 0;
    }
  }
}

Usb2CanRouter::~Usb2CanRouter() { stop(); }

int Usb2CanRouter::find_dev_index_(Usb2CanDevice* dev) const {
  for (int i=0; i<(int)devs_.size(); ++i) {
    if (devs_[i] == dev) return i;
  }
  return -1;
}

void Usb2CanRouter::attach_motor(RobStrideMotor* m)
{
  if (running_.load(std::memory_order_relaxed)) {
    std::cerr << "[ATTACH][WARN] attach_motor called while running. Prefer attach before start.\n";
  }

  const auto& b = m->bus();
  const uint8_t ch  = (uint8_t)b.channel;
  const uint8_t mid = (uint8_t)m->motor_id();

  if (ch < 1 || ch > 2) {
    std::cerr << "[ATTACH][ERR] invalid channel=" << int(ch) << " (expect 1/2)\n";
    return;
  }

  // ✅ 不依赖 b.dev_idx，直接用 bus.dev 指针在 devs_ 里找真实 index
  int dev_idx = find_dev_index_(b.dev);
  if (dev_idx < 0) {
    std::cerr << "[ATTACH][ERR] bus.dev ptr not found in devs_. "
              << "bus.dev=" << b.dev << "\n";
    return;
  }

  table_[dev_idx][ch][mid] = m;

  // round sync expected
  sync_[dev_idx].bus[ch].expected.set(mid);
  sync_[dev_idx].bus[ch].finalize_expected();

  std::cout << "[ATTACH] dev_idx=" << dev_idx
            << " dev_path=" << devs_[dev_idx]->path()
            << " ch=" << int(ch)
            << " motor_id=" << int(mid)
            << " motor_name=" << m->motorName_
            << "\n";
}

void Usb2CanRouter::start()
{
  stat_.clear();
  stat_last_.clear();
  stat_.resize(devs_.size());
  stat_last_.resize(devs_.size());
  for (size_t di=0; di<devs_.size(); ++di) {
    stat_[di].reset();
    stat_last_[di].reset();
    for (int ch=1; ch<=2; ++ch) {
      sync_[di].bus[ch].finalize_expected();
      sync_[di].bus[ch].reset_round();
    }
  }

  running_.store(true, std::memory_order_relaxed);

  threads_.clear();
  threads_.reserve(devs_.size());
  for (int di=0; di<(int)devs_.size(); ++di) {
    threads_.emplace_back([this, di]{ rx_loop(di); });
  }

  last_print_ = std::chrono::steady_clock::now();
}

void Usb2CanRouter::stop()
{
  running_.store(false, std::memory_order_relaxed);
  for (auto& t : threads_) {
    if (t.joinable()) t.join();
  }
  threads_.clear();
}

void Usb2CanRouter::rx_loop(int dev_idx)
{
  Usb2CanFrame f{};
  auto* dev = devs_[dev_idx];

  // ✅ Tangair 风格：阻塞 1s
  while (running_.load(std::memory_order_relaxed))
  {
    bool ok = dev->recv(f, 1000000); // 1s
    if (!ok) {
      stat_[dev_idx].timeouts.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    const uint8_t ch  = (uint8_t)f.channel;
    const uint8_t mid = (uint8_t)((f.eid >> 8) & 0xFF);

    if (ch < 1 || ch > 2) continue;

    stat_[dev_idx].raw[ch].fetch_add(1, std::memory_order_relaxed);

    RobStrideMotor* m = table_[dev_idx][ch][mid];
    if (m) {
      m->on_frame(f);
      stat_[dev_idx].dispatched[ch].fetch_add(1, std::memory_order_relaxed);

      auto& bs = sync_[dev_idx].bus[ch];
      bs.mark(mid);
      if (bs.round_complete()) {
        stat_[dev_idx].cycles[ch].fetch_add(1, std::memory_order_relaxed);
        bs.reset_round();
      }
    } else {
      stat_[dev_idx].nomatch[ch].fetch_add(1, std::memory_order_relaxed);
    }

    static std::atomic<int> dbg{0};
    if (dbg.fetch_add(1, std::memory_order_relaxed) < 10) {
      std::cout << "[RX] dev_idx=" << dev_idx
                << " dev_path=" << dev->path()
                << " ch=" << int(ch)
                << " eid=0x" << std::hex << f.eid << std::dec
                << " mid=" << int(mid)
                << (m ? " -> dispatch\n" : " -> nomatch\n");
    }
  }
}

void Usb2CanRouter::print_periodic()
{
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_print_).count();
  if (dt < 1.0) return;
  last_print_ = now;

  std::cout << "----- Router RX dt=" << std::fixed << std::setprecision(3) << dt << "s -----\n";
  for (size_t di=0; di<devs_.size(); ++di)
  {
    auto* dev = devs_[di];

    uint64_t tmo  = stat_[di].timeouts.load(std::memory_order_relaxed);
    uint64_t tmo0 = stat_last_[di].timeouts.load(std::memory_order_relaxed);
    uint64_t dtmo = tmo - tmo0;
    stat_last_[di].timeouts.store(tmo, std::memory_order_relaxed);

    std::cout << "dev[" << di << "] " << dev->path()
              << " timeouts=" << dtmo << " (" << (double(dtmo)/dt) << " Hz)\n";

    for (int ch=1; ch<=2; ++ch)
    {
      uint64_t raw   = stat_[di].raw[ch].load(std::memory_order_relaxed);
      uint64_t disp  = stat_[di].dispatched[ch].load(std::memory_order_relaxed);
      uint64_t nom   = stat_[di].nomatch[ch].load(std::memory_order_relaxed);
      uint64_t cyc   = stat_[di].cycles[ch].load(std::memory_order_relaxed);

      uint64_t raw0  = stat_last_[di].raw[ch].load(std::memory_order_relaxed);
      uint64_t disp0 = stat_last_[di].dispatched[ch].load(std::memory_order_relaxed);
      uint64_t nom0  = stat_last_[di].nomatch[ch].load(std::memory_order_relaxed);
      uint64_t cyc0  = stat_last_[di].cycles[ch].load(std::memory_order_relaxed);

      uint64_t draw  = raw  - raw0;
      uint64_t ddisp = disp - disp0;
      uint64_t dnom  = nom  - nom0;
      uint64_t dcyc  = cyc  - cyc0;

      stat_last_[di].raw[ch].store(raw, std::memory_order_relaxed);
      stat_last_[di].dispatched[ch].store(disp, std::memory_order_relaxed);
      stat_last_[di].nomatch[ch].store(nom, std::memory_order_relaxed);
      stat_last_[di].cycles[ch].store(cyc, std::memory_order_relaxed);

      double nom_pct = (draw > 0) ? (100.0 * double(dnom) / double(draw)) : 0.0;
      double cyc_hz  = double(dcyc) / dt;

      int expected = (int)sync_[di].bus[ch].expected_cnt;

      std::cout << "  bus" << ch
                << " expected=" << expected
                << " raw=" << draw << " (" << (double(draw)/dt) << " Hz)"
                << " disp=" << ddisp
                << " nomatch=" << dnom << " (" << std::setprecision(2) << nom_pct << "%)"
                << " cycles=" << dcyc << " (" << std::setprecision(3) << cyc_hz << " Hz)"
                << "\n";
      std::cout << std::setprecision(3);
    }
  }
}
