#include "GridRobot.hpp"

#include <filesystem>
#include <iostream>
#include <thread>
#include <cmath>

static void motorState_update_state_from_motor(RobStrideMotor* m, MotorState& s)
{
  auto pvtt_opt = m->last_pvtt();
  if (!pvtt_opt) {
    s.online = false;
    return;
  }
  const auto& pvtt = *pvtt_opt;
  s.online = true;
  s.q    = pvtt.p;
  s.dq   = pvtt.v;
  s.tau  = pvtt.t;
  s.temp = pvtt.temp;
  s.age_ms = 0;
}

static void jointState_update_state_from_motor(RobStrideMotor* m, JointState& j)
{
  auto pvtt_opt = m->last_pvtt();
  if (!pvtt_opt) return;
  const auto& pvtt = *pvtt_opt;
  j.q_joint   = pvtt.p;
  j.dq_joint  = pvtt.v;
  j.tau_joint = pvtt.t;
}

static void db_apply_cmd_to_motor(RobStrideMotor* m, const MotorCmd& c)
{
  if (!c.enable || c.mode == CtrlMode::DISABLE) {
    m->send_motion_command(0.f, 0.f, 0.f, 0.f, 0.f);
    return;
  }
  if (c.mode == CtrlMode::MOTION) {
    m->send_motion_command(c.tau_ff, c.q_des, c.dq_des, c.kp, c.kd);
    return;
  }
  if (c.mode == CtrlMode::TORQUE) {
    m->send_motion_command(c.tau_ff, 0.f, 0.f, 0.f, 0.f);
    return;
  }
  m->send_motion_command(0.f, 0.f, 0.f, 0.f, 0.f);
}

GridRobot::~GridRobot() {
  try {
    if (args_.dry_run) {
      imu_runner_.stop();          // dry-run也停IMU线程
    } else {
      stopAll_();                  // stopAll_里已经 imu_runner_.stop()
      if (router_) router_->stop();
    }
  } catch (...) {}
  closeLog_();
}

GridRobot::Args GridRobot::parse_args(int argc, char** argv) {
  Args a;
  for (int i=1;i<argc;i++) {
    std::string k = argv[i];
    auto need = [&](const std::string& name){
      if (i+1>=argc) { std::cerr<<"missing value for "<<name<<"\n"; std::exit(1); }
      return std::string(argv[++i]);
    };
    if (k=="--config") a.config = need(k);
    else if (k=="--only-group") a.only_group_csv = need(k);
    else if (k=="--rate") a.rate_hz = std::stoi(need(k));
    else if (k=="--test") a.test = need(k);
    else if (k=="--amp") a.sine_amp = std::stof(need(k));
    else if (k=="--freq") a.sine_freq = std::stof(need(k));
    else if (k=="--kp") a.kp = std::stof(need(k));
    else if (k=="--kd") a.kd = std::stof(need(k));
    else if (k=="--dry-run") a.dry_run = true;
    else if (k=="--imu-enable") a.imu_enable = true;
    else if (k=="--imu-only") { a.imu_enable = true; a.imu_only = true; }
    else if (k=="--imu-port") a.imu_port = need(k);
    else if (k=="--imu-baud") a.imu_baud = std::stoi(need(k));
    else if (k=="--telemetry-hz") a.telemetry_hz = std::stoi(need(k));

    else if (k=="--help") {
      std::cout <<
        "Usage:\n"
        "  ./motor_test --config config/motors.yaml \n"
        "              [--only-group u1b1,u2b2]\n"
        "              [--rate 1000]\n"
        "              [--test damping|sine|hold|fkik]\n"
        "              [--amp 0.3 --freq 0.5]\n"
        "              [--kp 20 --kd 1]\n"
        "              [--dry-run]   (load yaml + select motors, but DO NOT open /dev/USB2CAN*)\n";
      std::exit(0);
    }
  }
  return a;
}

bool GridRobot::group_selected(const std::string& group, const std::vector<std::string>& only_groups) {
  if (only_groups.empty()) return true;
  for (auto& g : only_groups) if (g == group) return true;
  return false;
}

bool GridRobot::init(const Args& args) {
  args_ = args;
  only_groups_ = split_csv(args_.only_group_csv);

  std::cout << "Loading config: " << args_.config << "\n";
  cfg_ = load_config_yaml(args_.config);

  ankleP_ = ankle::make_params_v2();

  ankleP_.invert_motor1 = true;
  ankleP_.invert_motor2 = false;

  dt_us_ = int(1e6 / std::max(1, args_.rate_hz));
  next_ = std::chrono::steady_clock::now();
  t0_   = std::chrono::steady_clock::now();

  if (args_.dry_run) return initDryRun();
  return initHardware();
}

bool GridRobot::initDryRun()
{
  // 1) 先选电机
  dry_selected_.clear();
  dry_selected_.reserve(64);

  for (int di=0; di<(int)cfg_.devices.size(); ++di) {
    auto& devSpec = cfg_.devices[di];
    for (auto& bus : devSpec.buses) {
      for (auto& ms : bus.motors) {
        bool sel = ms.enabled && group_selected(ms.group, only_groups_);
        if (!sel) continue;

        MotorKey mk;
        mk.dev_idx    = di;
        mk.channel    = bus.channel;
        mk.motor_id   = ms.id;
        mk.dev_name   = devSpec.name;
        mk.group      = ms.group;
        mk.type_str   = actuator_to_string(ms.type);
        mk.motor_name = ms.motorName;

        int sidx = stats_.register_motor(mk);
        dry_selected_.push_back(Sel{mk, sidx});
      }
    }
  }

  std::cout << "[DRY-RUN] Selected motors: " << dry_selected_.size() << "\n";
  if (dry_selected_.empty()) {
    if (!(args_.imu_enable && args_.imu_only)) {
      std::cout << "[DRY-RUN] No motors selected. Check YAML enabled or --only-group.\n";
      return false;
    }
    std::cout << "[DRY-RUN] No motors selected (imu-only mode).\n";
    // 允许继续
  } else {
    for (auto& s : dry_selected_) {
      std::cout << "  " << s.mk.motor_name << "  "
                << s.mk.dev_name << ":ch" << int(s.mk.channel) << ":id" << int(s.mk.motor_id)
                << " type=" << s.mk.type_str << " group=" << s.mk.group << "\n";
    }

    dry_name2dummy_.clear();
    dry_name2dummy_.reserve(dry_selected_.size());
    for (int i=0;i<(int)dry_selected_.size();++i) {
      dry_name2dummy_[dry_selected_[i].mk.motor_name] = i;
    }
  }

  // 2) 再创建 databus（size 与选中电机一致；imu-only 时 size=0 也没问题）
  databus_ = DataBus((int)dry_selected_.size());
  databus_.disable_all();

  // 3) 最后启动 IMU（dry-run 也启动）
  if (args_.imu_enable) {
    WitMotionImuRunner::Options opt;
    opt.port = args_.imu_port;
    opt.baud = args_.imu_baud; // 默认921600
    imu_runner_.start(opt);
  }

  return true;
}

bool GridRobot::initHardware() {
  // 1) need_dev
  need_dev_.assign(cfg_.devices.size(), false);
  for (int di=0; di<(int)cfg_.devices.size(); ++di) {
    auto& devSpec = cfg_.devices[di];
    for (auto& bus : devSpec.buses) {
      for (auto& ms : bus.motors) {
        bool sel = ms.enabled && group_selected(ms.group, only_groups_);
        if (sel) { need_dev_[di] = true; break; }
      }
      if (need_dev_[di]) break;
    }
  }

  // 2) open only needed devices
  dev_ptrs_.assign(cfg_.devices.size(), nullptr);
  dev_objs_.clear();
  dev_objs_.reserve(cfg_.devices.size());

  for (int di=0; di<(int)cfg_.devices.size(); ++di) {
    auto& d = cfg_.devices[di];
    if (!need_dev_[di]) {
      std::cout << "Skip open " << d.name << " (not selected)\n";
      continue;
    }
    auto dev = std::make_unique<Usb2CanDevice>(d.path);
    std::cout << "Opened " << d.name << " at " << d.path << "\n";
    dev_ptrs_[di] = dev.get();
    dev_objs_.push_back(std::move(dev));
  }

  // 3) build motors
  motors_.clear();
  active_.clear();
  kp_by_idx_.clear();
  kd_by_idx_.clear();

  kp_by_idx_.reserve(64);
  kd_by_idx_.reserve(64);

  for (int di=0; di<(int)cfg_.devices.size(); ++di) {
    auto& devSpec = cfg_.devices[di];
    for (auto& bus : devSpec.buses) {
      for (auto& ms : bus.motors) {
        bool sel = ms.enabled && group_selected(ms.group, only_groups_);
        if (!sel) continue;

        BusHandle bh;
        bh.dev_idx  = di;
        bh.dev_name = devSpec.name;
        bh.dev      = dev_ptrs_[di];
        bh.channel  = bus.channel;

        MotorKey mk;
        mk.dev_idx    = di;
        mk.channel    = bus.channel;
        mk.motor_id   = ms.id;
        mk.dev_name   = devSpec.name;
        mk.group      = ms.group;
        mk.type_str   = actuator_to_string(ms.type);
        mk.motor_name = ms.motorName;

        int sidx = stats_.register_motor(mk);

        motors_.push_back(std::make_unique<RobStrideMotor>(
          bh, cfg_.master_id, ms.id, ms.type, ms.group, &stats_, sidx, ms.motorName
        ));

        kp_by_idx_.push_back(ms.kp);
        kd_by_idx_.push_back(ms.kd);
      }
    }
  }

  for (auto& m : motors_) active_.push_back(m.get());

  std::cout << "Active motors: " << active_.size() << "\n";
  if (active_.empty()) {
    std::cout << "No motors selected. Check YAML enabled or --only-group.\n";
    return false;
  }

  // 4) start router
  std::vector<Usb2CanDevice*> opened_devs;
  opened_devs.reserve(cfg_.devices.size());
  for (auto* p : dev_ptrs_) if (p) opened_devs.push_back(p);

  router_ = std::make_unique<Usb2CanRouter>(opened_devs);
  for (auto* m : active_) router_->attach_motor(m);
  router_->start();

  // 5) build idx once
  buildIdxOnce_();
  buildJointIdxOnce_();
  //  6) IMU
  if(args_.imu_enable){
    WitMotionImuRunner::Options opt;
    opt.port = args_.imu_port;
    opt.baud = args_.imu_baud;
    imu_runner_.start(opt);
  }

  // 7) open log + databus
  openLog_();
  databus_ = DataBus((int)active_.size());
  databus_.disable_all();

  motorPos_rad_ = Eigen::VectorXd::Zero((int)active_.size());
  motorVel_rad_= Eigen::VectorXd::Zero((int)active_.size());
  motorTau_Nm_= Eigen::VectorXd::Zero((int)active_.size());
  motorCmdPos_rad_ = Eigen::VectorXd::Zero((int)active_.size());
  motorCmdVel_rad_ = Eigen::VectorXd::Zero((int)active_.size());

  jointPos_rad_ = Eigen::VectorXd::Zero((int)active_.size());
  jointVel_rad_ = Eigen::VectorXd::Zero((int)active_.size());
  jointTau_Nm_ = Eigen::VectorXd::Zero((int)active_.size());
  jointCmdPos_rad_ = Eigen::VectorXd::Zero((int)active_.size());
  jointCmdVel_rad_ = Eigen::VectorXd::Zero((int)active_.size());
  // 7) power on sequence
  powerOnSequence_();

  // reset timing
  next_ = std::chrono::steady_clock::now();
  t0_   = std::chrono::steady_clock::now();

  return true;
}

void GridRobot::buildIdxOnce_() {
  motor_idx_.clear();
  motor_idx_.reserve(active_.size());
  for (int i=0;i<(int)active_.size();++i) {
    motor_idx_[active_[i]->motorName_] = i;
  }
}

void GridRobot::buildJointIdxOnce_() {
  joint_names_.assign(active_.size(), "");
  for (int i=0;i<(int)active_.size();++i) joint_names_[i] = active_[i]->motorName_;

  // 把踝关节两电机对应的“关节语义名”改掉
  // 这里理解为joint和motor的name没有区别，在ankle上从左右变为pitch 和roll 
  auto itL1 = motor_idx_.find("L_ankle_L");
  auto itL2 = motor_idx_.find("L_ankle_R");
  if (itL1!=motor_idx_.end() && itL2!=motor_idx_.end()) {
    joint_names_[itL1->second] = "L_ankle_pitch";
    joint_names_[itL2->second] = "L_ankle_roll";
  }

  auto itR1 = motor_idx_.find("R_ankle_L");
  auto itR2 = motor_idx_.find("R_ankle_R");
  if (itR1!=motor_idx_.end() && itR2!=motor_idx_.end()) {
    joint_names_[itR1->second] = "R_ankle_pitch";
    joint_names_[itR2->second] = "R_ankle_roll";
  }

  joint_idx_.clear();
  joint_idx_.reserve(joint_names_.size());
  for (int i=0;i<(int)joint_names_.size();++i) joint_idx_[joint_names_[i]] = i;
}

void GridRobot::openLog_() {
  log_path_ = log_dir_ + "/track.csv";
  try {
    std::filesystem::create_directories(log_dir_);
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("[log] create_directories failed: ") + e.what());
  }

  std::cout << "[log] track.csv => " << log_path_ << "\n";
  flog_ = std::fopen(log_path_.c_str(), "w");
  if (!flog_) {
    std::perror("[log] open log file failed");
    throw std::runtime_error("open log file failed");
  }
  setvbuf(flog_, nullptr, _IOLBF, 0);
  std::fprintf(flog_, "t,i,q,dq,q_des,dq_des\n");
  std::fflush(flog_);
}

void GridRobot::closeLog_() {
  if (flog_) {
    std::fflush(flog_);
    std::fclose(flog_);
    flog_ = nullptr;
  }
}

void GridRobot::powerOnSequence_() {
  std::cout << "Init motors: stop -> set_mode(0) -> enable -> set_zero\n";
  for (auto* m : active_) {
    m->stop(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    m->set_mode(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    m->enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    for (int k=0;k<10;++k) {
      m->send_motion_command(0.0f, 0.0f, 0.0f, 0.0f, args_.kd);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    m->set_zero();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void GridRobot::stopAll_() {
  for (auto* m : active_) {
    m->stop(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  imu_runner_.stop();
}

void GridRobot::exec(std::atomic<bool>& run_flag) {
  if (args_.dry_run) execDryRun(run_flag);
  else execHardware(run_flag);
}

void GridRobot::execDryRun(std::atomic<bool>& run_flag) {
  auto has_name = [&](const char* n)->bool{
    return dry_name2dummy_.find(n) != dry_name2dummy_.end();
  };

  std::cout << "[DRY-RUN] Control loop running (no hardware). Ctrl+C to stop.\n";
  next_ = std::chrono::steady_clock::now();
  t0_   = std::chrono::steady_clock::now();

  while (run_flag.load()) {
    auto now = std::chrono::steady_clock::now();
    double t = std::chrono::duration<double>(now - t0_).count();

    if (args_.test == "fkik") {
      if (!has_name("L_ankle_L") || !has_name("L_ankle_R")) {
        static int once = 0;
        if (!once++) {
          std::cout << "[DRY-RUN][fkik] Missing motorName L_ankle_L / L_ankle_R in selected set.\n";
          std::cout << "Selected motorNames:\n";
          for (auto& s : dry_selected_) std::cout << "  " << s.mk.motor_name << "\n";
        }
      }

      // CLI amp 是 rad，这里转 deg 给运动学（运动学全 deg）
      constexpr double RAD2DEG = 180.0 / M_PI;
      double amp_deg = args_.sine_amp * RAD2DEG;
      double w = 2.0 * M_PI * args_.sine_freq;

      // FK -> IK
      double m1_deg = amp_deg * std::sin(w * t);
      double m2_deg = 0.8 * amp_deg * std::cos(w * t);
      auto fk1 = ankle::forwardKinematics(ankleP_, m1_deg, m2_deg, 0.0, 0.0, 80, 1e-10);

      double pitch_deg = NAN, roll_deg = NAN, e_m1 = NAN, e_m2 = NAN;
      if (fk1.ok) {
        pitch_deg = fk1.joint_deg.pitch_deg;
        roll_deg  = fk1.joint_deg.roll_deg;
        auto ik1 = ankle::inverseKinematics(ankleP_, pitch_deg, roll_deg);
        e_m1 = std::abs(ik1.sol_minus.m1_deg - m1_deg);
        e_m2 = std::abs(ik1.sol_minus.m2_deg - m2_deg);
      }

      // IK -> FK
      double pitch_des = 0.6 * amp_deg * std::sin(w * t);
      double roll_des  = 0.5 * amp_deg * std::cos(w * t);
      auto ik2 = ankle::inverseKinematics(ankleP_, pitch_des, roll_des);
      auto fk2 = ankle::forwardKinematics(ankleP_, ik2.sol_minus.m1_deg, ik2.sol_minus.m2_deg, 0.0, 0.0, 80, 1e-10);

      double e_pitch = NAN, e_roll = NAN;
      if (fk2.ok) {
        e_pitch = std::abs(fk2.joint_deg.pitch_deg - pitch_des);
        e_roll  = std::abs(fk2.joint_deg.roll_deg  - roll_des);
      }

      static int print_div = 0;
      if (++print_div >= std::max(1, args_.rate_hz/2)) {
        print_div = 0;
        std::cout
          << "[DRY-RUN][fkik] FK->IK m=(" << m1_deg << "," << m2_deg << ")deg"
          << " joint=(" << pitch_deg << "," << roll_deg << ")deg"
          << " err_m=(" << e_m1 << "," << e_m2 << ")deg"
          << " | IK->FK joint_des=(" << pitch_des << "," << roll_des << ")deg"
          << " err_joint=(" << e_pitch << "," << e_roll << ")deg\n";
      }
    }
    databusRead_();   // active_为空时不会读电机，但会更新 IMU -> databus_.imu()
    printStep_();     // 统一打印：IMU + motors(0个也OK的)
    stats_.print_periodic();
    next_ += std::chrono::microseconds(dt_us_);
    std::this_thread::sleep_until(next_);
  }
  if(args_.imu_enable) imu_runner_.stop();

  std::cout << "[DRY-RUN] Stopped.\n";
}

void GridRobot::execHardware(std::atomic<bool>& run_flag) {
  std::cout << "Control loop running. Ctrl+C to stop.\n";

  next_ = std::chrono::steady_clock::now();
  t0_   = std::chrono::steady_clock::now();

  while (run_flag.load()) {
    auto now = std::chrono::steady_clock::now();
    double t = std::chrono::duration<double>(now - t0_).count();

    databusRead_();
    ankleCalc_();
    databusWrite_(t);
    logStep_(t);
    applyCmd_();
    printStep_();

    stats_.print_periodic();
    next_ += std::chrono::microseconds(dt_us_);
    std::this_thread::sleep_until(next_);
  }

  std::cout << "Stopping...\n";
  stopAll_();
}

void GridRobot::databusRead_() {
  for (int i=0; i<(int)active_.size(); ++i) {
    motorState_update_state_from_motor(active_[i], databus_.motorState(i));
    jointState_update_state_from_motor(active_[i], databus_.jointState(i));
    motorPos_rad_(i) = databus_.motorState(i).q;
    motorVel_rad_(i) = databus_.motorState(i).dq;
    motorTau_Nm_(i) = databus_.motorState(i).tau;

    jointPos_rad_(i) = databus_.jointState(i).q_joint;
    jointVel_rad_(i) = databus_.jointState(i).dq_joint;
    jointTau_Nm_(i) = databus_.jointState(i).tau_joint;
  }
  auto &u = databus_.imu();
  ImuState s;
  uint32_t age_ms = 0;
  if (imu_runner_.get_latest(s, &age_ms))
  {
    u.online = true;
    u.seq = s.seq;

    u.ax = (float)s.accel_mps2.x();
    u.ay = (float)s.accel_mps2.y();
    u.az = (float)s.accel_mps2.z();

    u.gx = (float)s.gyro_rps.x();
    u.gy = (float)s.gyro_rps.y();
    u.gz = (float)s.gyro_rps.z();

    u.roll_deg = (float)s.rpy_deg.x();
    u.pitch_deg = (float)s.rpy_deg.y();
    u.yaw_deg = (float)s.rpy_deg.z();

    u.age_ms = age_ms;
  }
  else
  {
    u.online = false;
    u.age_ms = std::min<uint32_t>(u.age_ms + (uint32_t)(dt_us_ / 1000), 999999);
  }
}

void GridRobot::ankleCalc_() {
  // 写一个用于寻找motor和joint的lambda函数
  auto idx_motor = [&](const char* name)->int{
    auto it = motor_idx_.find(name);
    return (it == motor_idx_.end()) ? -1 : it->second;
  };
  auto idx_joint = [&](const char *name) -> int
  {
    auto it = joint_idx_.find(name);
    return (it == joint_idx_.end()) ? -1 : it->second;
  };
  // 计算一侧的电机的lambda函数
  auto calc_one = [&](const char *motorLeft, const char *motorRight,
                      const char *jointPitch, const char *jointRoll,
                      double &last_pitch_deg, double &last_roll_deg,
                      double &last_fkik_em1_deg, double &last_fkik_em2_deg)
  {
    const int idx_m1 = idx_motor(motorLeft);
    const int idx_m2 = idx_motor(motorRight);
    const int j_pitch = idx_joint(jointPitch);
    const int j_roll = idx_joint(jointRoll);
    if (idx_m1 < 0 || idx_m2 < 0 || j_pitch < 0 || j_roll < 0)
      return;
    const double m1_deg = ankle::rad2deg(motorPos_rad_(idx_m1));
    const double m2_deg = ankle::rad2deg(motorPos_rad_(idx_m2));
    auto fk = ankle::forwardKinematics(ankleP_, m1_deg, m2_deg, 0.0, 0.0, 80, 1e-10);
    if (!fk.ok) return;
    last_pitch_deg = fk.joint_deg.pitch_deg;
    last_roll_deg  = fk.joint_deg.roll_deg;

    // joint pos(rad)
    jointPos_rad_(j_pitch) = ankle::deg2rad(last_pitch_deg);
    jointPos_rad_(j_roll)  = ankle::deg2rad(last_roll_deg);
    databus_.jointState(j_pitch).q_joint = jointPos_rad_(j_pitch);
    databus_.jointState(j_roll ).q_joint = jointPos_rad_(j_roll);

    // joint vel(rad/s)
    Eigen::Vector2d qdot = ankle::fkVelocity_joint(
      ankleP_,
      jointPos_rad_(j_pitch),jointPos_rad_(j_roll),
      motorPos_rad_(idx_m1), motorPos_rad_(idx_m2),
      motorVel_rad_(idx_m1), motorVel_rad_(idx_m2)
    );
    jointVel_rad_(j_pitch) = qdot(0);
    jointVel_rad_(j_roll)  = qdot(1);

    // joint tau(Nm)
    Eigen::Vector2d tau_q = ankle::fkTorque_joint(
      ankleP_,
      jointPos_rad_(j_pitch), jointPos_rad_(j_roll),
      motorPos_rad_(idx_m1), motorPos_rad_(idx_m2),
      motorTau_Nm_(idx_m1), motorTau_Nm_(idx_m2),
      +1.0
    );
    jointTau_Nm_(j_pitch) = tau_q(0);
    jointTau_Nm_(j_roll ) = tau_q(1);
    databus_.jointState(j_pitch).tau_joint = tau_q(0);
    databus_.jointState(j_roll ).tau_joint = tau_q(1);

    // 检查连续性
    auto ik = ankle::inverseKinematics(ankleP_,last_pitch_deg,last_roll_deg);
    if(ik.ok) {
      auto err = [&](const decltype(ik.sol_minus)& sol){
        return std::abs(sol.m1_deg - m1_deg) + std::abs(sol.m2_deg - m2_deg);
      };
      const double e_minus = err(ik.sol_minus);
      const double e_plus  = err(ik.sol_plus);
      const auto& sol = (e_minus <= e_plus) ? ik.sol_minus : ik.sol_plus;
      last_fkik_em1_deg = std::abs(sol.m1_deg - m1_deg);
      last_fkik_em2_deg = std::abs(sol.m2_deg - m2_deg);
    } else {
      last_fkik_em1_deg = NAN;
      last_fkik_em2_deg = NAN;
    }
  };
  // // ---- motor index：用于取Ankle电机状态 ----
  // auto it1 = motor_idx_.find("L_ankle_L");
  // auto it2 = motor_idx_.find("L_ankle_R");
  // if (it1 == motor_idx_.end() || it2 == motor_idx_.end()) return;
  // const int im1 = it1->second;
  // const int im2 = it2->second;
  //  // ---- joint index：用于写 pitch/roll 的关节状态（jointName list）----
  // auto jp_it = joint_idx_.find("L_ankle_pitch");
  // auto jr_it = joint_idx_.find("L_ankle_roll");
  // if (jp_it == joint_idx_.end() || jr_it == joint_idx_.end()) return;
  // const int j_pitch = jp_it->second;
  // const int j_roll  = jr_it->second;
  // // motordata Fetch(rad, rad/s, Nm)
  // const double m1_rad = motorPos_rad_(im1);
  // const double m2_rad = motorPos_rad_(im2);
  // // const double v1_rad = motorVel_rad_(im1);
  // // const double v2_rad = motorVel_rad_(im2);
  // // const double t1_Nm  = motorTau_Nm_(im1);
  // // const double t2_Nm  = motorTau_Nm_(im2);
  // // 转单位
  // double m1_deg = ankle::rad2deg(m1_rad);
  // double m2_deg = ankle::rad2deg(m2_rad);
  // auto fk = ankle::forwardKinematics(ankleP_, m1_deg, m2_deg, 0.0, 0.0, 80, 1e-10);
  // if (!fk.ok) return;
  // last_pitch_deg_ = fk.joint_deg.pitch_deg;
  // last_roll_deg_  = fk.joint_deg.roll_deg;
  // // 更新一下joint的Pos数据
  // jointPos_rad_(j_pitch) = ankle::deg2rad(last_pitch_deg_);
  // jointPos_rad_(j_roll)  = ankle::deg2rad(last_roll_deg_);
  // databus_.jointState(j_pitch).q_joint = jointPos_rad_(j_pitch);
  // databus_.jointState(j_roll).q_joint  = jointPos_rad_(j_roll);
  // // 计算并更新Vel
  // Eigen::Vector2d qdot = ankle::fkVelocity_joint(
  //   ankleP_, 
  //   jointPos_rad_(j_pitch),
  //   jointPos_rad_(j_roll),
  //   motorPos_rad_(im1),
  //   motorPos_rad_(im2),
  //   motorVel_rad_(im1),
  //   motorVel_rad_(im2)
  // );
  // jointVel_rad_(j_pitch) = qdot(0);
  // jointVel_rad_(j_roll) = qdot(1);
  // databus_.jointState(j_pitch).dq_joint = qdot(0);
  // databus_.jointState(j_roll).dq_joint = qdot(1);
  // // 计算并更新Tau
  // // const double tau_m1 = motorTau_Nm_(im1);
  // // const double tau_m2 = motorTau_Nm_(im2);
  // Eigen::Vector2d tau_q = ankle::fkTorque_joint(
  //   ankleP_,
  //   jointPos_rad_(j_pitch),
  //   jointPos_rad_(j_roll),
  //   motorPos_rad_(im1),
  //   motorPos_rad_(im2),
  //   motorTau_Nm_(im1),
  //   motorTau_Nm_(im2), +1.0);
  // jointTau_Nm_(j_pitch) = tau_q(0);
  // jointTau_Nm_(j_roll) = tau_q(1);
  // databus_.jointState(j_pitch).tau_joint = tau_q(0);
  // databus_.jointState(j_roll).tau_joint = tau_q(1);
  // // FK->IK 自洽误差
  // auto ik = ankle::inverseKinematics(ankleP_, last_pitch_deg_, last_roll_deg_);
  // if (ik.ok)
  // {
  //   auto err = [&](const decltype(ik.sol_minus) &sol)
  //   {
  //     return std::abs(sol.m1_deg - m1_deg) + std::abs(sol.m2_deg - m2_deg);
  //   };
  //   const double e_minus = err(ik.sol_minus);
  //   const double e_plus = err(ik.sol_plus);
  //   const auto &sol = (e_minus <= e_plus) ? ik.sol_minus : ik.sol_plus;
  //   last_fkik_em1_deg_ = std::abs(sol.m1_deg - m1_deg);
  //   last_fkik_em2_deg_ = std::abs(sol.m2_deg - m2_deg);
  // }
  // else
  // {
  //   last_fkik_em1_deg_ = NAN;
  //   last_fkik_em2_deg_ = NAN;
  // }
  calc_one("L_ankle_L", "L_ankle_R", 
           "L_ankle_pitch", "L_ankle_roll",
           last_L_pitch_deg_, last_L_roll_deg_, 
           last_L_fkik_em1_deg_, last_L_fkik_em2_deg_);

  calc_one("R_ankle_L", "R_ankle_R", 
           "R_ankle_pitch", "R_ankle_roll",
           last_R_pitch_deg_, last_R_roll_deg_, 
           last_R_fkik_em1_deg_, last_R_fkik_em2_deg_);

}

void GridRobot::databusWrite_(double t)
{
  // clear
  motorCmdPos_rad_.setZero();
  motorCmdVel_rad_.setZero();
  jointCmdPos_rad_.setZero();
  jointCmdVel_rad_.setZero();
  // find Left ankle motor idx
  int leftMotorIdx1 = -1, leftMotorIdx2 = -1;
  {
    auto it = motor_idx_.find("L_ankle_L");
    if (it != motor_idx_.end())
      leftMotorIdx1 = it->second;
    it = motor_idx_.find("L_ankle_R");
    if (it != motor_idx_.end())
      leftMotorIdx2 = it->second;
  }

  // find Right ankle motor idx
  int rightMotorIdx1 = -1, rightMotorIdx2 = -1;
  {
    auto it = motor_idx_.find("R_ankle_L");
    if (it != motor_idx_.end())
      rightMotorIdx1 = it->second;
    it = motor_idx_.find("R_ankle_R");
    if (it != motor_idx_.end())
      rightMotorIdx2 = it->second;
  }

  // find Left ankle joint idx
  int lj_pitch = -1, lj_roll = -1;
  {
    auto it = joint_idx_.find("L_ankle_pitch");
    if (it != joint_idx_.end())
      lj_pitch = it->second;
    it = joint_idx_.find("L_ankle_roll");
    if (it != joint_idx_.end())
      lj_roll = it->second;
  }

  // find Right ankle joint idx
  int rj_pitch = -1, rj_roll = -1;
  {
    auto it = joint_idx_.find("R_ankle_pitch");
    if (it != joint_idx_.end())
      rj_pitch = it->second;
    it = joint_idx_.find("R_ankle_roll");
    if (it != joint_idx_.end())
      rj_roll = it->second;
  }

  auto is_ankle_motor = [&](int i)
  {
    return (i == leftMotorIdx1 || i == leftMotorIdx2 || i == rightMotorIdx1 || i == rightMotorIdx2);
  };

  for (int i=0;i<(int)active_.size();++i) {

    databus_.motorCmd(i) = MotorCmd{};

    if (is_ankle_motor(i) && args_.test != "damping") continue;

    auto& motorCmd = databus_.motorCmd(i);
    
    if (args_.test == "damping") {
      motorCmd.enable = true;
      motorCmd.mode   = CtrlMode::MOTION;
      motorCmd.tau_ff = 0.f;
      motorCmd.q_des  = 0.f;
      motorCmd.dq_des = 0.f;
      motorCmd.kp = 0.f;
      motorCmd.kd = kd_by_idx_[i];
    }
    else if (args_.test == "hold") {
      motorCmd.enable = true;
      motorCmd.mode   = CtrlMode::MOTION;
      motorCmd.tau_ff = 0.f;
      motorCmd.q_des  = 0.f;
      motorCmd.dq_des = 0.f;
      motorCmd.kp     = kp_by_idx_[i];
      motorCmd.kd     = kd_by_idx_[i];
    }
    else if (args_.test == "sine")
    {
      auto &sp = databus_.sine(i);
      if (!sp.armed && databus_.motorState(i).online)
      {
        sp.amp_rad = args_.sine_amp;
        sp.freq_hz = args_.sine_freq;
        sp.kp = kp_by_idx_[i];
        sp.kd = kd_by_idx_[i];
        sp.q0 = databus_.motorState(i).q; // motor q0
        sp.armed = true;
      }
      if (!sp.armed)
      {
        motorCmd = MotorCmd{};
      }
      else
      {
        const double w = 2.0 * M_PI * (double)sp.freq_hz;
        motorCmd.enable = true;
        motorCmd.mode = CtrlMode::MOTION;
        motorCmd.tau_ff = 0.f;
        motorCmd.q_des = sp.q0 + sp.amp_rad * std::sin(w * t);
        motorCmd.dq_des = sp.amp_rad * w * std::cos(w * t);
        motorCmd.kp = sp.kp;
        motorCmd.kd = sp.kd;
      }
    }
    else
    {
      motorCmd = MotorCmd{}; // disable
    }
    // 记录cmd数据到motorCmd里面
    // jointCmdPos_rad_(i) = motorCmd.q_des;
    // jointCmdVel_rad_(i) = motorCmd.dq_des;
    // 记录到 motorCmd 观测缓存
    motorCmdPos_rad_(i) = motorCmd.q_des;
    motorCmdVel_rad_(i) = motorCmd.dq_des;
  }
  if (args_.test == "damping")
  {
    // damping 时不做 IK 控制踝
    return;
  }
  // 进行IK 依旧用lambda函数 joint_des -> IK -> motorCmd
  auto apply_ankle_ik = [&](int im1, int im2, int j_pitch, int j_roll)
  {
    if (im1 < 0 || im2 < 0 || j_pitch < 0 || j_roll < 0)
      return;

    double pitch_des = 0.0, roll_des = 0.0;
    double pitch_d_des = 0.0, roll_d_des = 0.0;

    if (args_.test == "hold")
    {
      pitch_des = 0.0;
      roll_des = 0.0;
      pitch_d_des = 0.0;
      roll_d_des = 0.0;
    }
    else if (args_.test == "sine")
    {
      auto &sp_p = databus_.sine(j_pitch);
      auto &sp_r = databus_.sine(j_roll);

      if (!sp_p.armed)
      {
        sp_p.amp_rad = args_.sine_amp;
        sp_p.freq_hz = args_.sine_freq;
        sp_p.q0 = (float)jointPos_rad_(j_pitch);
        sp_p.armed = true;
      }
      if (!sp_r.armed)
      {
        sp_r.amp_rad = args_.sine_amp;
        sp_r.freq_hz = args_.sine_freq;
        sp_r.q0 = (float)jointPos_rad_(j_roll);
        sp_r.armed = true;
      }

      const double w = 2.0 * M_PI * (double)args_.sine_freq;
      pitch_des = sp_p.q0 + sp_p.amp_rad * std::sin(w * t);
      pitch_d_des = sp_p.amp_rad * w * std::cos(w * t);

      roll_des = sp_r.q0 + sp_r.amp_rad * std::cos(w * t);
      roll_d_des = -sp_r.amp_rad * w * std::sin(w * t);
    }
    else
    {
      databus_.motorCmd(im1) = MotorCmd{};
      databus_.motorCmd(im2) = MotorCmd{};
      return;
    }
    // 写 jointCmd 
    jointCmdPos_rad_(j_pitch) = pitch_des;
    jointCmdVel_rad_(j_pitch) = pitch_d_des;
    jointCmdPos_rad_(j_roll ) = roll_des;
    jointCmdVel_rad_(j_roll ) = roll_d_des;

    databus_.jointCmd(j_pitch).q_joint_des  = (float)pitch_des;
    databus_.jointCmd(j_pitch).dq_joint_des = (float)pitch_d_des;
    databus_.jointCmd(j_roll ).q_joint_des  = (float)roll_des;
    databus_.jointCmd(j_roll ).dq_joint_des = (float)roll_d_des;

    // IK (deg)

    auto ik = ankle::inverseKinematics(
      ankleP_, 
      ankle::rad2deg(pitch_des), 
      ankle::rad2deg(roll_des));
    if (!ik.ok)
    {
      databus_.motorCmd(im1) = MotorCmd{};
      databus_.motorCmd(im2) = MotorCmd{};
      return;
    }
    // 选更近的解，避免 sol_plus/sol_minus 跳变
    const double m1_deg_cur = ankle::rad2deg(motorPos_rad_(im1));
    const double m2_deg_cur = ankle::rad2deg(motorPos_rad_(im2));
    auto err = [&](const decltype(ik.sol_minus)& s){
      return std::abs(s.m1_deg - m1_deg_cur) + std::abs(s.m2_deg - m2_deg_cur);
    };
    const auto& sol = (err(ik.sol_minus) <= err(ik.sol_plus)) ? ik.sol_minus : ik.sol_plus;

    const double m1_des_rad = ankle::deg2rad(sol.m1_deg);
    const double m2_des_rad = ankle::deg2rad(sol.m2_deg);

    // 速度：joint_dot -> motor_dot

    Eigen::Vector2d mdot = ankle::ikVelocity_motor(
        ankleP_, 
        jointPos_rad_(j_pitch), 
        jointPos_rad_(j_roll), 
        motorPos_rad_(im1), 
        motorPos_rad_(im2),
        pitch_d_des, 
        roll_d_des);

    // 更新下发到两个踝电机的 motorCmd（引用)
    auto& c1 = databus_.motorCmd(im1);
    c1.enable = true;
    c1.mode   = CtrlMode::MOTION;
    c1.tau_ff = 0.f;
    c1.q_des  = (float)m1_des_rad;
    c1.dq_des = (float)mdot(0);
    c1.kp = kp_by_idx_[im1];
    // c1.kp     = (args_.test == "damping") ? 0.f : kp_by_idx_[im1];
    c1.kd     = kd_by_idx_[im1];

    auto& c2 = databus_.motorCmd(im2);
    c2.enable = true;
    c2.mode   = CtrlMode::MOTION;
    c2.tau_ff = 0.f;
    c2.q_des  = (float)m2_des_rad;
    c2.dq_des = (float)mdot(1);
    // c2.kp     = (args_.test == "damping") ? 0.f : kp_by_idx_[im2];
    c2.kp = kp_by_idx_[im2];
    c2.kd = kd_by_idx_[im2];

    // 记录到 motorCmd 观测缓存 应该可以直接删了
    // motorCmdPos_rad_(im1) = c1.q_des;
    // motorCmdVel_rad_(im1) = c1.dq_des;
    // motorCmdPos_rad_(im2) = c2.q_des;
    // motorCmdVel_rad_(im2) = c2.dq_des;
  };
  apply_ankle_ik(leftMotorIdx1,leftMotorIdx2, lj_pitch,lj_roll);
  apply_ankle_ik(rightMotorIdx1, rightMotorIdx2, rj_pitch, rj_roll);

  // 记录一下本周期 command（用于后续 log/观测）
  for (int i = 0; i < (int)active_.size(); ++i)
  {
    motorCmdPos_rad_(i) = databus_.motorCmd(i).q_des;
    motorCmdVel_rad_(i) = databus_.motorCmd(i).dq_des;
  }
}

void GridRobot::applyCmd_() {
  for (int i=0;i<(int)active_.size();++i) {
    db_apply_cmd_to_motor(active_[i], databus_.motorCmd(i));
  }
}

void GridRobot::logStep_(double t) {
  if (!flog_) return;

  static int log_div = 0;
  if (++log_div < log_every_) return;
  log_div = 0;

  for (int i=0;i<(int)active_.size();++i) {
    const auto& s = databus_.motorState(i);
    const auto& c = databus_.motorCmd(i);
    std::fprintf(flog_, "%.6f,%d,%.6f,%.6f,%.6f,%.6f\n",
                 t, i,
                 (double)s.q, (double)s.dq,
                 (double)c.q_des, (double)c.dq_des);
  }

  static int flush_batches = 0;
  if (++flush_batches >= 100) {
    flush_batches = 0;
    std::fflush(flog_);
  }
}

void GridRobot::printStep_()
{
  static int print_div = 0;
  int div = std::max(1, args_.rate_hz / std::max(1, args_.telemetry_hz));
  if (++print_div >= div)
  {
    print_div = 0;

    databus_.print_telemetry_line("Telemetry(databus)");

    if (!std::isnan(last_L_pitch_deg_))
    {
      std::cout << "[L_ankle] pitch=" << last_L_pitch_deg_
                << " roll=" << last_L_roll_deg_
                << " fkik_err_m(deg)=(" << last_L_fkik_em1_deg_ << "," << last_L_fkik_em2_deg_ << ")\n";
    }
    if (!std::isnan(last_R_pitch_deg_))
    {
      std::cout << "[R_ankle] pitch=" << last_R_pitch_deg_
                << " roll=" << last_R_roll_deg_
                << " fkik_err_m(deg)=(" << last_R_fkik_em1_deg_ << "," << last_R_fkik_em2_deg_ << ")\n";
    }
  }
}
