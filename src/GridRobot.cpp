#include "GridRobot.hpp"

#include <filesystem>
#include <iostream>
#include <thread>
#include <cmath>

#ifndef GRIDROBOT_SINE_TEST
#define GRIDROBOT_SINE_TEST 1
#endif

// static void jointState_update_state_from_motor(RobStrideMotor* m, JointState& j)
// {
//   auto pvtt_opt = m->last_pvtt();
//   if (!pvtt_opt) return;
//   const auto& pvtt = *pvtt_opt;
//   j.q_joint   = pvtt.p;
//   j.dq_joint  = pvtt.v;
//   j.tau_joint = pvtt.t;
// }
static inline double clamp01(double x) {
  if (x < 0.0) return 0.0;
  if (x > 1.0) return 1.0;
  return x;
}
static inline double smooth5(double a) {
  // minimum-jerk: s = 10a^3 - 15a^4 + 6a^5
  return a*a*a*(10.0 + a*(-15.0 + 6.0*a));
}
static inline double smooth5d(double a) {
  // ds/da = 30a^2 - 60a^3 + 30a^4
  return 30.0*a*a - 60.0*a*a*a + 30.0*a*a*a*a;
}
static inline float clipf(float x, float lo, float hi) {
  return std::min(std::max(x, lo), hi);
}
static void applyCmdToMotorTemplate(RobStrideMotor* m, const MotorCmd& c)
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

GridRobot::~GridRobot() {
  try {
    if (args_.dry_run) {
      imu_runner_.stop();          
    }
    else if (args_.cheat_run){
      imu_runner_.stop();
    } 
    else {
      stopAll_();
      if (router_) router_->stop();
    }
  } catch (...) {}
  stopPolicyIoLog_();
  closeLog_();
}

GridRobot::Args GridRobot::parse_args(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; i++)
  {
    std::string k = argv[i];
    auto need = [&](const std::string &name)
    {
      if (i + 1 >= argc)
      {
        std::cerr << "missing value for " << name << "\n";
        std::exit(1);
      }
      return std::string(argv[++i]);
    };
    if (k == "--config")
      a.config = need(k);
    else if (a.dry_run && a.cheat_run)
    {
      std::cerr << "ERROR: --dry-run and --cheat-run are mutually exclusive.\n";
      std::exit(1);
    }
    else if (k=="--only-group") a.only_group_csv = need(k);
    else if (k=="--rate") a.rate_hz = std::stoi(need(k));
    else if (k=="--test") a.test = need(k);
    else if (k=="--kp") a.kp = std::stof(need(k));
    else if (k=="--kd") a.kd = std::stof(need(k));
    else if (k=="--dry-run") a.dry_run = true;
    else if (k=="--cheat-run") a.cheat_run = true;
    else if (k=="--imu-enable") a.imu_enable = true;
    else if (k=="--imu-only") { a.imu_enable = true; a.imu_only = true; }
    else if (k=="--imu-port") a.imu_port = need(k);
    else if (k=="--imu-baud") a.imu_baud = std::stoi(need(k));
    else if (k=="--telemetry-hz") a.telemetry_hz = std::stoi(need(k));
    else if (k=="--policy") a.policy_path = need(k);
    else if (k=="--frame-stack") a.frame_stack = std::stoi(need(k));
    else if (k=="--decimation") a.decimation = std::stoi(need(k));
    else if (k=="--action-scale") a.action_scale = std::stof(need(k));
    else if (k=="--clip-actions") a.clip_actions = std::stof(need(k));
    else if (k=="--clip-obs") a.clip_obs = std::stof(need(k));
    else if (k=="--gait-period") a.gait_period = std::stof(need(k));
    else if (k=="--cmd-vx") a.cmd_vx = std::stof(need(k));
    else if (k=="--cmd-vy") a.cmd_vy = std::stof(need(k));
    else if (k=="--cmd-dyaw") a.cmd_dyaw = std::stof(need(k));
    // scales
    else if (k=="--obs-lin") a.obs_lin_vel_scale = std::stof(need(k));
    else if (k=="--obs-ang") a.obs_ang_vel_scale = std::stof(need(k));
    else if (k=="--obs-pos") a.obs_dof_pos_scale = std::stof(need(k));
    else if (k=="--obs-vel") a.obs_dof_vel_scale = std::stof(need(k));
    else if (k=="--warmup") a.warmup_steps = std::stoi(need(k));
    // zero
    else if (k=="--skip-zero-sta") a.skip_zero_sta = true;
    else if (k=="--skip-set-zero") a.skip_set_zero = true;
    else if (k=="--skip-zero") { a.skip_zero_sta = true; a.skip_set_zero = true; }
    else if (k=="--no-verify-zero") a.verify_zero_on_boot = false;



    else if (k=="--help") {
      std::cout <<
        "Usage:\n"
        "  ./motor_test --config config/motors.yaml \n"
        "              [--only-group u1b1,u2b2]\n"
        "              [--rate 1000]\n"
        "              [--test damping|hold|fkik|policy]\n"
        "              [--kp 20 --kd 1]\n"
        "              [--dry-run]   (load yaml + select motors, but DO NOT open /dev/USB2CAN*)\n"
        "              [--cheat-run] (NO /dev/USB2CAN*, fake motor/joint/imu states as YAML defaults)\n"
        "              [--skip-zero-sta] (do NOT write 0x7029 zero_sta; keep motor's saved value)\n"
        "              [--skip-set-zero] (do NOT call set_zero; keep motor's saved zero)\n"
        "              [--skip-zero]     (same as --skip-zero-sta --skip-set-zero)\n"
        "              [--no-verify-zero] (when skipping, do not print boot pos verification)\n";
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

  if (args_.dry_run) return initDryRun();
  if (args_.cheat_run) return initCheatRun_();
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
  default_motor_angle_rad_vec.clear();
  default_joint_angle_rad_vec.clear();
  kp_by_idx_.reserve(64);
  kd_by_idx_.reserve(64);
  default_motor_angle_rad_vec.reserve(64);
  default_joint_angle_rad_vec.reserve(64);
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
        default_motor_angle_rad_vec.push_back(ms.default_motor_angle_rad);
        default_joint_angle_rad_vec.push_back(ms.default_joint_angle_rad);
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
  initAnkleIdxOnce_();
  initPolicyJointMapAndDefaults_();

  policy_gate_state_ = 0;
  policy_gate_prompted_ = false;
  policy_run_steps_ = 0;
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
  // 8) Policy
  // ===== policy init =====
  if (!args_.policy_path.empty())
  {
    try
    {
      policy_ = torch::jit::load(args_.policy_path);
      policy_.eval();
      policy_ok_ = true;
      std::cout << "[policy] loaded: " << args_.policy_path << "\n";
    }
    catch (const c10::Error &e)
    {
      std::cerr << "[policy] load failed: " << e.what() << "\n";
      policy_ok_ = false;
    }
  }

  // ===== obs stack init =====
  hist_obs_.clear();
  for (int i = 0; i < std::max(1, args_.frame_stack); ++i)
  {
    std::array<float, kNumSingleObs> z{};
    z.fill(0.f);
    hist_obs_.push_back(z);
  }
  policy_input_.assign(args_.frame_stack * kNumSingleObs, 0.f);

  last_action_.fill(0.f);
  action_.fill(0.f);
  lowlevel_cnt_ = 0;

  // 9) power on sequence
  powerOnSequence_();

  // reset timing
  next_ = std::chrono::steady_clock::now();

  return true;
}

bool GridRobot::initCheatRun_()
{
  std::cout << "[CHEAT-RUN] init (NO hardware IO). All states = YAML defaults.\n";

  // 1) build "motors_" and "active_" exactly like initHardware, but WITHOUT opening devices/router
  motors_.clear();
  active_.clear();
  kp_by_idx_.clear();
  kd_by_idx_.clear();
  default_motor_angle_rad_vec.clear();
  default_joint_angle_rad_vec.clear();

  kp_by_idx_.reserve(64);
  kd_by_idx_.reserve(64);
  default_motor_angle_rad_vec.reserve(64);
  default_joint_angle_rad_vec.reserve(64);

  for (int di=0; di<(int)cfg_.devices.size(); ++di) {
    auto& devSpec = cfg_.devices[di];
    for (auto& bus : devSpec.buses) {
      for (auto& ms : bus.motors) {
        bool sel = ms.enabled && group_selected(ms.group, only_groups_);
        if (!sel) continue;

        // IMPORTANT: dev=nullptr, because we won't send/recv anything in cheat-run
        BusHandle bh;
        bh.dev_idx  = di;
        bh.dev_name = devSpec.name;
        bh.dev      = nullptr;
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
        default_motor_angle_rad_vec.push_back(ms.default_motor_angle_rad);
        default_joint_angle_rad_vec.push_back(ms.default_joint_angle_rad);
      }
    }
  }

  for (auto& m : motors_) active_.push_back(m.get());

  std::cout << "[CHEAT-RUN] Active motors: " << active_.size() << "\n";
  if (active_.empty()) {
    std::cout << "[CHEAT-RUN] No motors selected. Check YAML enabled or --only-group.\n";
    return false;
  }

  // 2) build indices (same as hardware)
  buildIdxOnce_();
  buildJointIdxOnce_();
  initAnkleIdxOnce_();
  initPolicyJointMapAndDefaults_();

  policy_gate_state_ = 0;
  policy_gate_prompted_ = false;
  policy_run_steps_ = 0;

  // 3) databus & scratch
  openLog_(); // optional: keep same logging behavior
  databus_ = DataBus((int)active_.size());
  databus_.disable_all();

  const int n = (int)active_.size();
  motorPos_rad_      = Eigen::VectorXd::Zero(n);
  motorVel_rad_      = Eigen::VectorXd::Zero(n);
  motorTau_Nm_       = Eigen::VectorXd::Zero(n);
  motorCmdPos_rad_   = Eigen::VectorXd::Zero(n);
  motorCmdVel_rad_   = Eigen::VectorXd::Zero(n);

  jointPos_rad_ = Eigen::VectorXd::Zero(n);
  jointVel_rad_ = Eigen::VectorXd::Zero(n);
  jointTau_Nm_ = Eigen::VectorXd::Zero(n);
  jointCmdPos_rad_ = Eigen::VectorXd::Zero(n);
  jointCmdVel_rad_ = Eigen::VectorXd::Zero(n);

  for (int i = 0; i < n; ++i)
  {
    float qm = (i < (int)default_motor_angle_rad_vec.size()) ? default_motor_angle_rad_vec[i] : 0.f;
    motorPos_rad_(i) = qm;
    motorVel_rad_(i) = 0.0;
    motorTau_Nm_(i) = 0.0;
  }

  // 4) policy load (same as hardware, optional)
  policy_ok_ = false;
  if (!args_.policy_path.empty()) {
    try {
      policy_ = torch::jit::load(args_.policy_path);
      policy_.eval();
      policy_ok_ = true;
      std::cout << "[policy] loaded: " << args_.policy_path << "\n";
    } catch (const c10::Error &e) {
      std::cerr << "[policy] load failed: " << e.what() << "\n";
      policy_ok_ = false;
    }
  }

  hist_obs_.clear();
  for (int i = 0; i < std::max(1, args_.frame_stack); ++i) {
    std::array<float, kNumSingleObs> z{}; z.fill(0.f);
    hist_obs_.push_back(z);
  }
  policy_input_.assign(args_.frame_stack * kNumSingleObs, 0.f);
  last_action_.fill(0.f);
  action_.fill(0.f);
  lowlevel_cnt_ = 0;

  // reset timing
  next_ = std::chrono::steady_clock::now();

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

  auto jointIdx = [&](const std::string& n)->int{
  auto it = joint_idx_.find(n);
  return it==joint_idx_.end() ? -1 : it->second;
};

jidx_L_knee_ = jointIdx("L_knee");
jidx_R_knee_ = jointIdx("R_knee");

// 校验：缺失 or 重名覆盖（会导致 policy_joint_idx_ 指错）
auto countName = [&](const std::string& n)->int{
  int c = 0;
  for (auto &s : joint_names_) if (s == n) ++c;
  return c;
};

int cL = countName("L_knee");
int cR = countName("R_knee");
if (cL != 1 || cR != 1 || jidx_L_knee_ < 0 || jidx_R_knee_ < 0) {
  std::string msg = "[init] knee joint name invalid: "
    "count(L_knee)=" + std::to_string(cL) + " idx=" + std::to_string(jidx_L_knee_) + ", "
    "count(R_knee)=" + std::to_string(cR) + " idx=" + std::to_string(jidx_R_knee_) + "\n"
    "=> check YAML motorName uniqueness / spelling.";
  throw std::runtime_error(msg);
}

std::cout << "[kneeIdx] L_knee jointIdx=" << jidx_L_knee_
          << "  R_knee jointIdx=" << jidx_R_knee_ << "\n";

}

void GridRobot::openLog_() {
  log_path_ = log_dir_ + "/obs_compare.csv";
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
  std::fprintf(flog_,
  "t,i,"
  "q_m,dq_m,"          // motor state
  "q_m_des,dq_m_des,"  // motor cmd (post mapping)
  "q_j,dq_j,"          // joint semantic state
  "q_j_des,dq_j_des,"  // joint semantic cmd (pre mapping snapshot)
  "motor_name,joint_name\n"
);
  std::fflush(flog_);
}

void GridRobot::closeLog_() {
  if (flog_) {
    std::fflush(flog_);
    std::fclose(flog_);
    flog_ = nullptr;
  }
}

void GridRobot::powerOnSequence_()
{
  using namespace std::chrono;

  // ====== tunables ======
  // const int   kEnableWaitMs        = 1000;  // enable 后等状态反馈
  // const int   kWarmupMs            = 60;    // enable 后先阻尼一段时间
  // const int   kSetZeroRetries      = 5;     // set_zero 重试次数
  // const int   kSetZeroTimeoutMs    = 250;   // 每次 set_zero 后等待验证的超时
  // const int   kPollMs              = 5;     // 轮询 last_pvtt 的间隔
  // const int   kConsecutiveOkNeed   = 3;     // 连续N次读到“接近0”才算成功
  // const float kZeroEpsRad          = 0.25f; // 判定“已归零”的阈值（rad），按你电机噪声可调
  const int kEnableWaitMs = 1000;
  const int kWarmupMs = 60;
  const int kPollMs = 5;

  // zero_sta barrier
  const uint8_t kZeroStaValue = 1;   // 1: (-pi,pi]   0: [0,2pi)
  const bool kZeroStaPersist = true; // 写flash（Type22）
  const int kZeroStaTimeoutMs = 120; // 单次 get_param 等待
  const int kZeroStaRetries = 3;

  // set_zero verify
  const int kSetZeroRetries = 5;
  const int kSetZeroTimeoutMs = 250;
  const int kConsecutiveOkNeed = 3;
  const float kZeroEpsRad = 0.25f;

  auto sleep_ms = [](int ms)
  { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); };

  auto disableMotor = [&](RobStrideMotor *m)
  {
    try
    {
      m->send_motion_command(0.f, 0.f, 0.f, 0.f, 0.f);
    }
    catch (...)
    {
    }
    try
    {
      m->stop(0);
    }
    catch (...)
    {
    }
  };

  auto readPos = [&](RobStrideMotor *m, float &p_out) -> bool
  {
    auto pvtt_opt = m->last_pvtt();
    if (!pvtt_opt)
      return false;
    p_out = pvtt_opt->p;
    return true;
  };

  auto waitOnline = [&](RobStrideMotor *m, int timeout_ms) -> bool
  {
    auto t0 = steady_clock::now();
    while (duration_cast<milliseconds>(steady_clock::now() - t0).count() < timeout_ms)
    {
      float p = 0.f;
      if (readPos(m, p))
        return true;
      sleep_ms(kPollMs);
    }
    return false;
  };

  auto waitZeroed = [&](RobStrideMotor *m, int timeout_ms, float eps_rad) -> bool
  {
    auto t0 = steady_clock::now();
    int consecutive_ok = 0;

    while (duration_cast<milliseconds>(steady_clock::now() - t0).count() < timeout_ms)
    {
      float p = 0.f;
      if (readPos(m, p))
      {
        if (std::fabs(p) <= eps_rad)
        {
          if (++consecutive_ok >= kConsecutiveOkNeed)
            return true;
        }
        else
        {
          consecutive_ok = 0;
        }
      }
      sleep_ms(kPollMs);
    }
    return false;
  };
  struct FailInfo
  {
    std::string name;
    std::string reason;
  };
  std::vector<FailInfo> fails;

  // std::cout << "Init motors: stop -> set_mode(0) -> enable -> damping -> set_zero(verify)\n";

  // ---------- Phase A ----------
  std::cout << "[power] PhaseA: stop -> set_mode(0) -> enable -> waitOnline\n";
  fails.clear();

  for (auto *m : active_)
  {
    const std::string name = m->motorName_;

    try
    {
      // 0) stop / mode / enable
      m->stop(0);
      sleep_ms(2);
      m->set_mode(0);
      sleep_ms(2);
      m->enable();
      sleep_ms(2);

      // 1) 等到电机有状态回报
      if (!waitOnline(m, kEnableWaitMs))
      {
        disableMotor(m);
        fails.push_back({name, "no status feedback after enable (last_pvtt empty)"});
        std::cerr << "[power] FAIL " << name << ": no pvtt after enable, disabled.\n";
        continue;
      }

      // 2) warmup damping to stabilize feedback stream
      const int warmup_iters = std::max(1, kWarmupMs); // 1ms 一次
      for (int k = 0; k < warmup_iters; ++k)
      {
        m->send_motion_command(0.f, 0.f, 0.f, 0.f, args_.kd);
        sleep_ms(1);
      }
    }
    catch (...)
    {
      disableMotor(m);
      fails.push_back({name, "exception in PhaseA"});
      std::cerr << "[power] FAIL " << name << ": exception in PhaseA\n";
    }
  }

  auto printBootPositions = [&]()
  {
    if (!args_.verify_zero_on_boot)
      return;

    std::cout << "[power] Boot positions (after enable+warmup):\n";
    float pmin = 1e9f, pmax = -1e9f;
    for (auto *m : active_)
    {
      float p = 0.f;
      bool ok = readPos(m, p);
      if (!ok)
      {
        std::cout << "  " << m->motorName_ << " : (no pvtt)\n";
        continue;
      }
      pmin = std::min(pmin, p);
      pmax = std::max(pmax, p);
      std::cout << "  " << m->motorName_ << " : p=" << p << " rad\n";
    }
    if (pmin < 1e8f)
    {
      std::cout << "[power] Boot pos range: [" << pmin << ", " << pmax << "] rad\n";
      std::cout << "        (expect near 0 if saved zero works; range hints zero_sta mode)\n";
    }
  };

  // ---------- Phase B: zero_sta ----------
  if (args_.skip_zero_sta)
  {
    std::cout << "[power] PhaseB: SKIP set_zero_sta (use motor saved zero_sta)\n";
    // 跳过时建议打印一下刚上电的位置范围，帮助判断 zero_sta 是否是你想要的
    printBootPositions();
  }
  else
  {
    std::cout << "[power] PhaseB: set zero_sta(0x7029)=" << int(kZeroStaValue)
              << " persist=" << (kZeroStaPersist ? 1 : 0)
              << " (ALL motors must OK)\n";
    fails.clear();

    for (auto *m : active_)
    {
      const std::string name = m->motorName_;
      bool ok = false;
      for (int attempt = 1; attempt <= kZeroStaRetries; ++attempt)
      {
        ok = m->set_zero_sta(kZeroStaValue, kZeroStaPersist, kZeroStaTimeoutMs);
        if (ok)
          break;
        std::cerr << "[power] WARN " << name
                  << " set_zero_sta attempt " << attempt << "/" << kZeroStaRetries
                  << " failed.\n";
        sleep_ms(10);
      }
      if (!ok)
      {
        disableMotor(m);
        fails.push_back({name, "set_zero_sta failed (no reply or readback mismatch)"});
        std::cerr << "[power] FAIL " << name << ": set_zero_sta failed, disabled.\n";
      }
      else
      {
        std::cout << "[power] OK " << name << " zero_sta=" << int(kZeroStaValue) << "\n";
      }
    }

    if (!fails.empty())
    {
      for (auto *m : active_)
        disableMotor(m);
      std::string msg = "[power] PhaseB(zero_sta) failed motors:\n";
      for (auto &f : fails)
        msg += "  - " + f.name + " : " + f.reason + "\n";
      throw std::runtime_error(msg);
    }
  }

  // ---------- Phase C: mechanical zero (set_zero) ----------
  // ---------- Phase C: mechanical zero (set_zero) ----------
  if (args_.skip_set_zero)
  {
    std::cout << "[power] PhaseC: SKIP set_zero (use motor saved zero)\n";
    // 再打印一次
    printBootPositions();

    if (args_.verify_zero_on_boot)
    {
      const float warn_eps = 0.35f; // 可以用 kZeroEpsRad
      for (auto *m : active_)
      {
        float p = 0.f;
        if (readPos(m, p) && std::fabs(p) > warn_eps)
        {
          std::cerr << "[power] WARN " << m->motorName_
                    << " boot pos |p|=" << std::fabs(p)
                    << " > " << warn_eps
                    << " rad (saved zero may NOT match current mechanical)\n";
        }
      }
    }
  }
  else
  {
    std::cout << "[power] PhaseC: set_zero(verify)\n";
    fails.clear();

    for (auto *m : active_)
    {
      const std::string name = m->motorName_;
      bool ok = false;

      for (int attempt = 1; attempt <= kSetZeroRetries; ++attempt)
      {
        m->set_zero();
        sleep_ms(5);

        // after set_zero keep sending damping a bit
        for (int k = 0; k < 20; ++k)
        {
          m->send_motion_command(0.f, 0.f, 0.f, 0.f, args_.kd);
          sleep_ms(1);
        }

        ok = waitZeroed(m, kSetZeroTimeoutMs, kZeroEpsRad);
        if (ok)
          break;

        float plast = 0.f;
        (void)readPos(m, plast);
        std::cerr << "[power] WARN " << name
                  << " set_zero attempt " << attempt << "/" << kSetZeroRetries
                  << " failed. last_pos=" << plast << "\n";
      }

      if (!ok)
      {
        float plast = 0.f;
        (void)readPos(m, plast);
        disableMotor(m);
        fails.push_back({name, "set_zero failed, last_pos=" + std::to_string(plast)});
        std::cerr << "[power] FAIL " << name << ": set_zero failed, disabled.\n";
      }
      else
      {
        std::cout << "[power] OK " << name << " zeroed.\n";
      }
    }

    if (!fails.empty())
    {
      for (auto *m : active_)
        disableMotor(m);
      std::string msg = "[power] PhaseC(set_zero) failed motors:\n";
      for (auto &f : fails)
        msg += "  - " + f.name + " : " + f.reason + "\n";
      throw std::runtime_error(msg);
    }
  }
  // 让 hold ramp 从下一次 writeHoldCmd 开始计时
  hold_ramp_t0_sec_ = -1.0;
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
  else if (args_.cheat_run) execCheatRun_(run_flag);
  else execHardware(run_flag);
}

void GridRobot::execDryRun(std::atomic<bool>& run_flag) {
  auto has_name = [&](const char* n)->bool{
    return dry_name2dummy_.find(n) != dry_name2dummy_.end();
  };

  std::cout << "[DRY-RUN] Control loop running (no hardware). Ctrl+C to stop.\n";
  auto now0 = std::chrono::steady_clock::now();
  next_ = now0;
  resetProgramTime_(now0);


  while (run_flag.load()) {
    auto now = std::chrono::steady_clock::now();
    updateTime_(now);
    const auto& tm = time_();
    double t = tm.program_sec;

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
  auto now0   = std::chrono::steady_clock::now();
  next_ = now0;
  resetProgramTime_(now0);

  while (run_flag.load()) {
    auto now = std::chrono::steady_clock::now();
    updateTime_(now);
    const auto& tm = time_();
    databusRead_();
    checkLimit_();
    if (emergency_damping_)
    {
      writeAllDamping_();
    }
    else
    {
      if (args_.test == "policy" && policy_gate_state_ == 2)
      {
        policyStep_(tm);
      }
      databusWrite_(tm);
    }
    logStep_(tm);
    applyCmd_();
    printStep_();

    // stats_.print_periodic();
    // router_->print_periodic();
    next_ += std::chrono::microseconds(dt_us_);
    std::this_thread::sleep_until(next_);
  }

  std::cout << "Stopping...\n";
  stopAll_();
}

void GridRobot::execCheatRun_(std::atomic<bool>& run_flag)
{
  std::cout << "[CHEAT-RUN] Control loop running (NO hardware IO). Ctrl+C to stop.\n";

  auto now0 = std::chrono::steady_clock::now();
  next_   = now0;
  resetProgramTime_(now0);

  while (run_flag.load()) {
    auto now = std::chrono::steady_clock::now();
    updateTime_(now);
    const auto& tm = time_();

    // 1) fake sensor readings use motor2joint
    cheatRunFunc_();
    motor2joint_mapping();
    // 2) policy inference if needed (only when gate RUN)
    if (args_.test == "policy" && policy_gate_state_ == 2) {
      policyStep_(tm);
    }

    // 3) generate commands
    if (args_.test == "hold") {
      cheatHoldDebugStep_(tm);   // pre/post mapping snapshot + finalize
    } else {
      databusWrite_(tm);        // normal path
    }
    // cheatPlantStep_(t);

    // 4) log/print (NO applyCmd_ !)
    logStep_(tm);
    printStep_();

    stats_.print_periodic();
    next_ += std::chrono::microseconds(dt_us_);
    std::this_thread::sleep_until(next_);
  }

  std::cout << "[CHEAT-RUN] Stopped.\n";
}

void GridRobot::cheatRunFunc_()
{
  const int n = (int)active_.size();

  // motor state = default motor angle
  for (int i=0;i<n;++i) {
    const float qm  = (float)motorPos_rad_(i);
    // const float dqm = (float)motorVel_rad_(i);
    // const float tau = (float)motorTau_Nm_(i);
    motorPos_rad_(i) = default_motor_angle_rad_vec[i];
    motorVel_rad_(i) = 0.0;
    motorTau_Nm_(i)  = 0.0;

    auto &ms = databus_.motorState(i);
    ms.online = true;
    ms.q = qm;
    ms.dq = 0.f;
    ms.tau = 0.f;
    ms.temp = 28.f;
    ms.age_ms = 0;
  }

  // joint state = default joint angle
  // for (int i=0;i<n;++i) {
  //   const float qj = (i < (int)default_joint_angle_rad_vec.size()) ? default_joint_angle_rad_vec[i] : 0.f;

  //   jointPos_rad_(i) = qj;
  //   jointVel_rad_(i) = 0.0;
  //   jointTau_Nm_(i)  = 0.0;

  //   auto &js = databus_.jointState(i);
  //   js.q_joint   = qj;
  //   js.dq_joint  = 0.f;
  //   js.tau_joint = 0.f;
  // }

  // imu = initial pose
  {
    auto &u = databus_.imu();
    u.online = true;
    u.seq += 1;
    u.ax = 0.f; u.ay = 0.f; u.az = 9.81f;
    u.gx = 0.f; u.gy = 0.f; u.gz = 0.f;
    u.roll_deg  = 0.f;
    u.pitch_deg = 0.f;
    u.yaw_deg   = 0.f;
    u.age_ms = 0;
  }

  // ankle telemetry (so printStep_ keeps meaningful)
  if (ankleL_.ok()) {
    last_L_pitch_deg_ = ankle::rad2deg(jointPos_rad_(ankleL_.jp));
    last_L_roll_deg_  = ankle::rad2deg(jointPos_rad_(ankleL_.jr));
  }
  if (ankleR_.ok()) {
    last_R_pitch_deg_ = ankle::rad2deg(jointPos_rad_(ankleR_.jp));
    last_R_roll_deg_  = ankle::rad2deg(jointPos_rad_(ankleR_.jr));
  }
}

void GridRobot::databusRead_() {
  for (int i=0; i<(int)active_.size(); ++i) {
    motorState_update_state_from_motor(active_[i], databus_.motorState(i));
    motorPos_rad_(i) = databus_.motorState(i).q;
    motorVel_rad_(i) = databus_.motorState(i).dq;
    motorTau_Nm_(i) = databus_.motorState(i).tau;
  }

  motor2joint_mapping();

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

void GridRobot::databusWrite_(const TimeInfo& tm) {
  clearAllMotorCmd_();

  if (args_.test == "damping") {
    writeDampingCmd_();
  } else if (args_.test == "hold") {
    writeHoldCmd_(tm);      // 用 tm.program_sec 做 hold ramp
    joint2motor_mapping();
  } else if (args_.test == "policy") {
    writePolicyCmd_(tm);    // gate + RUN 
    joint2motor_mapping();
  } else {
  }

  finalizeCmd();
}

void GridRobot::applyCmd_() {
  for (int i=0;i<(int)active_.size();++i) {
    applyCmdToMotorTemplate(active_[i], databus_.motorCmd(i));
  }
}

void GridRobot::logStep_(const TimeInfo& tm) {
  if (!flog_) return;

  static int log_div = 0;
  if (++log_div < log_every_) return;
  log_div = 0;

  for (int i=0;i<(int)active_.size();++i) {
    const auto& ms = databus_.motorState(i);
    const auto& mc = databus_.motorCmd(i);     // post-mapping motor cmd
    const auto& js = databus_.jointState(i);   // semantic joint state

    const double qj_des  = (double)jointCmdPos_rad_(i);
    const double dqj_des = (double)jointCmdVel_rad_(i);
    const double t = tm.program_sec;
    std::fprintf(flog_,
      "%.6f,%d,"
      "%.6f,%.6f,"
      "%.6f,%.6f,"
      "%.6f,%.6f,"
      "%.6f,%.6f,"
      "\"%s\",\"%s\"\n",
      t, i,
      (double)ms.q, (double)ms.dq,
      (double)mc.q_des, (double)mc.dq_des,
      (double)js.q_joint, (double)js.dq_joint,
      qj_des, dqj_des,
      active_[i]->motorName_.c_str(),
      (i < (int)joint_names_.size() ? joint_names_[i].c_str() : "")
    );
  }

  static int flush_batches = 0;
  if (++flush_batches >= 100) {
    flush_batches = 0;
    std::fflush(flog_);
  }
}

void GridRobot::printStep_()
{
  // ---- loop dt stats (update EVERY call) ----
  const auto& tm = time_();

  static double last_loop_t = -1.0;
  static double dt_sum = 0.0;
  static double dt_min = 1e9;
  static double dt_max = 0.0;
  static int    dt_cnt = 0;

  if (last_loop_t >= 0.0) {
    const double dt = tm.program_sec - last_loop_t;
    if (dt > 0.0 && dt < 1.0) { // basic sanity
      dt_sum += dt;
      dt_min = std::min(dt_min, dt);
      dt_max = std::max(dt_max, dt);
      dt_cnt++;
    }
  }
  last_loop_t = tm.program_sec;

  // ---- telemetry divider ----
  static int print_div = 0;
  int div = std::max(1, args_.rate_hz / std::max(1, args_.telemetry_hz));
  if (++print_div < div) return;
  print_div = 0;

  // 只在 cheat-run / dry-run 打印才打印time信息
  if (args_.cheat_run || args_.dry_run) {
    const double expect_dt = dt_us_ * 1e-6;
    const double dt_avg = (dt_cnt > 0) ? (dt_sum / dt_cnt) : 0.0;

    std::ios::fmtflags f = std::cout.flags();
    std::streamsize p = std::cout.precision();

    std::cout.setf(std::ios::fixed);
    std::cout << std::setprecision(6);

    std::cout
      << "[time]"
      << " prog=" << tm.program_sec << "s"
      << " policy=" << tm.policy_sec << "s"
      << " gate=" << policy_gate_state_
      << " t0_policy=" << policy_t0_program_sec_
      << " loop_dt_avg=" << (dt_avg * 1e3) << "ms"
      << " (min=" << (dt_min * 1e3) << "ms"
      << ", max=" << (dt_max * 1e3) << "ms"
      << ", expect=" << (expect_dt * 1e3) << "ms)"
      << " lowlevel_cnt=" << lowlevel_cnt_
      << " run_steps=" << policy_run_steps_
      << "\n";

    std::cout.flags(f);
    std::cout.precision(p);

    // reset stats for next telemetry window
    dt_sum = 0.0;
    dt_min = 1e9;
    dt_max = 0.0;
    dt_cnt = 0;
  }

  // ---- your existing prints ----
  databus_.print_telemetry_line("Telemetry(databus)");

  if (!std::isnan(last_L_pitch_deg_)) {
    std::cout << "[L_ankle] pitch=" << last_L_pitch_deg_
              << " roll=" << last_L_roll_deg_ << "\n";
  }
  if (!std::isnan(last_R_pitch_deg_)) {
    std::cout << "[R_ankle] pitch=" << last_R_pitch_deg_
              << " roll=" << last_R_roll_deg_ << "\n";
  }
}


void GridRobot::policyStep_(const TimeInfo& tm)
{
  if (!policy_ok_) return;

  // 只在 decimation 步推理；否则保持 action_
  if (args_.decimation > 1 && (lowlevel_cnt_ % args_.decimation) != 0) return;

  // 1) 构造 single obs (47)
  std::array<float,kNumSingleObs> obs{};
  obs.fill(0.f);

  // phase
  
  const double T = std::max(1e-6, (double)args_.gait_period);
  const double phase = 2.0 * M_PI * (tm.policy_sec / T);
  obs[0] = float(std::sin(phase));
  obs[1] = float(std::cos(phase));

  // cmd
  obs[2] = args_.cmd_vx   * args_.obs_lin_vel_scale;
  obs[3] = args_.cmd_vy   * args_.obs_lin_vel_scale;
  obs[4] = args_.cmd_dyaw * args_.obs_ang_vel_scale;

  // dof pos/vel: 用 jointPos_rad_/jointVel_rad_（语义关节）
  // 假设你 active_ 顺序 L1..L6,R1..R6
  for (int i=0;i<kNumActions;++i) {
    int j = policy_joint_idx_[i];
    // float q  = (i < jointPos_rad_.size()) ? float(jointPos_rad_(i)) : 0.f;
    // float dq = (i < jointVel_rad_.size()) ? float(jointVel_rad_(i)) : 0.f;
    float q  = (j>=0) ? (float)jointPos_rad_(j) : 0.f;
    float dq = (j>=0) ? (float)jointVel_rad_(j) : 0.f;
    obs[5  + i] = (q - default_angle_rad_[i]) * args_.obs_dof_pos_scale;
    // std::cerr << "obs[5 + i]"<<i<<" = "<<obs[5  + i]<<"\n"; //打印出来是0，没问题
    
    obs[17 + i] = dq * args_.obs_dof_vel_scale;
    obs[29 + i] = last_action_[i];
  }

  // imu
  auto &u = databus_.imu();
  // omega (rad/s)
  obs[41] = u.gx;
  obs[42] = u.gy;
  obs[43] = u.gz;
  // euler (rad)
  obs[44] = float(u.roll_deg  * M_PI / 180.0);
  obs[45] = float(u.pitch_deg * M_PI / 180.0);
  obs[46] = float(u.yaw_deg   * M_PI / 180.0);

  // clip obs
  for (auto &v : obs) v = clipf(v, -args_.clip_obs, args_.clip_obs);

  // 2) push into stack
  hist_obs_.push_back(obs);
  while ((int)hist_obs_.size() > std::max(1,args_.frame_stack)) hist_obs_.pop_front();

  // 3) stack -> policy_input
  const int fs = std::max(1,args_.frame_stack);
  for (int i=0;i<fs;++i) {
    const auto &oi = hist_obs_[i];
    std::memcpy(policy_input_.data() + i*kNumSingleObs, oi.data(), sizeof(float)*kNumSingleObs);
  }

  // 4) forward
  torch::NoGradGuard ng;
  auto in = torch::from_blob(policy_input_.data(), {1, fs*kNumSingleObs}, torch::kFloat32).clone();
  auto out = policy_.forward({in}).toTensor().to(torch::kCPU);

  // 5) copy + clip
  for (int i=0;i<kNumActions;++i) {
    float a = out[0][i].item<float>();
    a = clipf(a, -args_.clip_actions, args_.clip_actions);
    action_[i] = a;
  }

  last_action_ = action_;
  logPolicyIoRow_(tm);   // <--- ADD (only runs when decimation inference happens)

}

void GridRobot::checkLimit_()
{
  if (emergency_damping_) return;

  for (int i=0;i<kNumActions && i < jointPos_rad_.size(); ++i) {
    int j = policy_joint_idx_[i];
    if (j < 0) continue;
    float q = float(jointPos_rad_(j));
    if (q < joint_limit_[i].lo || q > joint_limit_[i].hi) {
      emergency_damping_ = true;
      emergency_idx_ = i;
      emergency_val_ = q;
      std::cerr << "[EMG] JointLimit violated action_idx=" << i
          << " joint_idx=" << j
          << " name=" << joint_names_[j]
          << " q_joint=" << q
          << " limit=[" << joint_limit_[i].lo << "," << joint_limit_[i].hi << "]"
          << " q_motor=" << (float)motorPos_rad_(j)
          << "\n";
      return;
    }
  }
}

void GridRobot::writeAllDamping_()
{
  for (int i=0;i<(int)active_.size();++i) {
    auto &c = databus_.motorCmd(i);
    c = MotorCmd{};
    c.enable = true;
    c.mode   = CtrlMode::MOTION;
    c.tau_ff = 0.f;
    c.q_des  = 0.f;      
    c.dq_des = 0.f;
    c.kp     = 0.f;
    c.kd     = kd_by_idx_[i];
  }
}

void GridRobot::initAnkleIdxOnce_() {
  auto motorIdx = [&](const std::string& n)->int{
    auto it = motor_idx_.find(n);
    return it==motor_idx_.end() ? -1 : it->second;
  };
  auto jointIdx = [&](const std::string& n)->int{
    auto it = joint_idx_.find(n);
    return it==joint_idx_.end() ? -1 : it->second;
  };

  ankleL_.m1 = motorIdx("L_ankle_L");
  ankleL_.m2 = motorIdx("L_ankle_R");
  ankleL_.jp = jointIdx("L_ankle_pitch");
  ankleL_.jr = jointIdx("L_ankle_roll");

  ankleR_.m1 = motorIdx("R_ankle_L");
  ankleR_.m2 = motorIdx("R_ankle_R");
  ankleR_.jp = jointIdx("R_ankle_pitch");
  ankleR_.jr = jointIdx("R_ankle_roll");

  // kneeL_ = motorIdx("L_knee");
  std::cout << "[ankleIdx] L(ok="<<ankleL_.ok()<<") m=("<<ankleL_.m1<<","<<ankleL_.m2<<") j=("<<ankleL_.jp<<","<<ankleL_.jr<<")\n";
  std::cout << "[ankleIdx] R(ok="<<ankleR_.ok()<<") m=("<<ankleR_.m1<<","<<ankleR_.m2<<") j=("<<ankleR_.jp<<","<<ankleR_.jr<<")\n";
  // std::cout << "[kneeIdx] L_knee idx=" << kneeL_ << "\n";
}

void GridRobot::clearAllMotorCmd_() {
  motorCmdPos_rad_.setZero();
  motorCmdVel_rad_.setZero();
  jointCmdPos_rad_.setZero();
  jointCmdVel_rad_.setZero();
  for (int i=0;i<(int)active_.size();++i) databus_.motorCmd(i) = MotorCmd{};
}

void GridRobot::finalizeCmd() {
  for (int i=0;i<(int)active_.size();++i) {
    motorCmdPos_rad_(i) = databus_.motorCmd(i).q_des;
    motorCmdVel_rad_(i) = databus_.motorCmd(i).dq_des;
  }
}

void GridRobot::writeDampingCmd_() {
  writeAllDamping_();
}

void GridRobot::writeHoldCmd_(const TimeInfo& tm) {
  const double t = tm.program_sec;
  // 在hold开始时，记住“起始姿态”q0（语义关节坐标）
  static std::vector<float> q0;
  if (hold_ramp_t0_sec_ < 0.0) {
    hold_ramp_t0_sec_ = t;
    q0.assign(active_.size(), 0.f);
    for (int i = 0; i < (int)active_.size(); ++i) {
      // 用当前语义关节位置作为起点（databusRead_之后 jointPos_rad_ 已更新）
      q0[i] = (i < jointPos_rad_.size()) ? (float)jointPos_rad_(i) : 0.f;
    }
  }
  if (hold_ramp_t0_sec_ < 0.0) hold_ramp_t0_sec_ = t;
  const double T = std::max(0.05,hold_ramp_dur_sec_);
  double a = (t - hold_ramp_t0_sec_) / T;
  a = clamp01(a);
  const double s  = smooth5(a);
  const double sdot = smooth5d(a) / T;  // ds/dt

  for (int i=0; i<(int)active_.size(); ++i) {
    auto &c = databus_.motorCmd(i);
    c.enable = true; 
    c.mode = CtrlMode::MOTION;
    c.tau_ff = 0.f;
    const float q_target = 
    (i < (int)default_joint_angle_rad_vec.size()) 
    ? 
    default_joint_angle_rad_vec[i] : 0.f;

    const float q_start = (i < (int)q0.size()) ? q0[i] : 0.f;
    const float delta_q      = q_target - q_start;

    // smooth pos targets
    c.q_des = q_start + (float)(s * (double)delta_q);

    c.dq_des = (float)(sdot * (double)delta_q);

    c.kp = kp_by_idx_[i];
    c.kd = kd_by_idx_[i];

    jointCmdPos_rad_(i) = c.q_des;
    jointCmdVel_rad_(i) = c.dq_des;
  }
}

void GridRobot::writePolicyCmd_(const TimeInfo& tm) {
    // 计算 posErr 的 1-norm：sum |q - q_default| ，检查是否已经稳定在屈膝位置
  auto calc_Pos_Err_one_norm = [&]()->float{
    float s = 0.f;
    for (int i=0;i<kNumActions;++i) {
      int j = policy_joint_idx_[i];
      if (j < 0 || j >= jointPos_rad_.size()) continue;
      s += std::fabs((float)jointPos_rad_(j) - default_angle_rad_[i]);
    }
    return s;
  };
  if(!policy_ok_)
  {
    writeHoldCmd_(tm);
    return;
  }
  // -------- gate: 0 HOLD(default) -> 1 WAIT(Enter) -> 2 RUN(policy) --------
  if (policy_gate_state_ != 2)
  {
    writeHoldCmd_(tm);
    float err = calc_Pos_Err_one_norm();

    if (policy_gate_state_ == 0 && err < 0.1f)
    {
      policy_gate_state_ = 1;
      policy_gate_prompted_ = false; // 提示一次
    }

    if (policy_gate_state_ == 1)
    {
      if (!policy_gate_prompted_)
      {
        policy_gate_prompted_ = true;
        std::cout << "[policy] HOLD stable: ||posErr||1=" << err
                  << " < 0.1rad. Press ENTER to start POLICY...\n";
      }
      struct pollfd pfd;
      pfd.fd = STDIN_FILENO;
      pfd.events = POLLIN;
      pfd.revents = 0;
      int r = ::poll(&pfd, 1, 0); // 0ms，不阻塞控制环
      if (r > 0 && (pfd.revents & POLLIN))
      {
        std::string line;
        std::getline(std::cin, line); // 读掉回车
        policy_gate_state_ = 2;

        // 进入RUN：重置计数/历史，warmup从这里开始算
        policy_run_steps_ = 0;
        policy_t0_program_sec_ = tm.program_sec;
        std::cout << "[policy] RUN t0_program=" << policy_t0_program_sec_ << "s (policy_sec will start from ~0)\n";
        lowlevel_cnt_ = 0;
        action_.fill(0.f);
        last_action_.fill(0.f);

        hist_obs_.clear();
        for (int i = 0; i < std::max(1, args_.frame_stack); ++i)
        {
          std::array<float, kNumSingleObs> z{};
          z.fill(0.f);
          hist_obs_.push_back(z);
        }
        startPolicyIoLog_();
        std::cout << "[policy] ENTER received. Start POLICY (warmup="
                  << args_.warmup_steps << " steps).\n";
      }
    }
    return; // gate阶段结束，但仍在保持默认姿态
  }
  // ---------------- RUN(policy) ----------------
  std::array<float, kNumActions> q_des{};
#if GRIDROBOT_SINE_TEST

  const double T = std::max(1e-6, (double)args_.gait_period);
  const float sin_pos = std::sin(float(2.0 * M_PI * (tm.policy_sec / T)));
  const double warmup_sec = double(std::max(0, args_.warmup_steps)) / double(std::max(1, args_.rate_hz));
  const bool use_ref = (tm.policy_sec >= warmup_sec);

  float sin_pos_l = sin_pos;
  float sin_pos_r = sin_pos;

  // 左脚：sin_pos_l > -0.1 则置0
  if (sin_pos_l > -0.1f)
    sin_pos_l = 0.f;
  // 右脚：sin_pos_r <  0.1 则置0
  if (sin_pos_r < 0.1f)
    sin_pos_r = 0.f;

  const float scale_1 = 0.25f;
  const float scale_2 = 0.45f;
  const float scale_3 = 0.20f;

  // ref_dof_pos = default_angle
  std::array<float, kNumActions> ref = default_angle_rad_;

  // left
  ref[0] = -sin_pos_l * scale_1 + 0.25f; // L_hip_pitch
  ref[3] = sin_pos_l * scale_2 - 0.45f;  // L_knee
  ref[4] = sin_pos_l * scale_3 - 0.20f;  // L_ankle_pitch

  // right
  ref[6] = -sin_pos_r * scale_1 - 0.25f;  // R_hip_pitch
  ref[9] = sin_pos_r * scale_2 + 0.45f;   // R_knee
  ref[10] = -sin_pos_r * scale_3 - 0.20f; // R_ankle_pitch

  // python: if count_lowlevel>800 use ref else default
  
  q_des = use_ref ? ref : default_angle_rad_;
  for (int i=0;i<kNumActions;++i) {
    jointCmdPos_rad_(i) = q_des[i];
  }
  #else
  // warmup：RUN阶段前 warmup_steps，强制 action=0 -> 保持 default
  const bool warmup = (policy_run_steps_ < std::max(0, args_.warmup_steps));
  
  for (int i=0;i<kNumActions;++i) {
    // float a = (lowlevel_cnt_ > args_.warmup_steps) ? action_[i] : 0.f;
    float a = (warmup) ? 0.f : action_[i];
    q_des[i] = default_angle_rad_[i] + a * args_.action_scale;
    jointCmdPos_rad_(i) = q_des[i];
  }
  #endif
  // 先用 hold 填 enable/mode/kp/kd ,后面应该会有一套新的runningKp和runningKd
  writeHoldCmd_(tm);

  for (int i=0;i<kNumActions;++i) {
    int j = policy_joint_idx_[i];
    if (j < 0 || j >= (int)active_.size()) continue;

    auto &c = databus_.motorCmd(j);
    // c.enable = true; c.mode = CtrlMode::MOTION;
    // c.tau_ff = 0.f;
    // c.kp = kp_by_idx_[j];
    // c.kd = kd_by_idx_[j];writeHoldCmd_已经实现了对kp kd的设置
    c.q_des = q_des[i];
    c.dq_des = 0.f;
    jointCmdPos_rad_(i) = c.q_des;
  }
  ++policy_run_steps_;
  ++lowlevel_cnt_;
}

void GridRobot::initPolicyJointMapAndDefaults_()
{
  static constexpr std::array<const char*, kNumActions> kPolicyJoints = {
    "L_hip_pitch","L_hip_roll","L_hip_yaw","L_knee","L_ankle_pitch","L_ankle_roll",
    "R_hip_pitch","R_hip_roll","R_hip_yaw","R_knee","R_ankle_pitch","R_ankle_roll"
  };

  for (int i=0;i<kNumActions;++i) {
    auto it = joint_idx_.find(kPolicyJoints[i]);
    policy_joint_idx_[i] = (it==joint_idx_.end()) ? -1 : it->second;
  }

  for (int i=0;i<kNumActions;++i) {
    int j = policy_joint_idx_[i];
    if (j >= 0 && j < (int)default_joint_angle_rad_vec.size()) {
      default_angle_rad_[i] = default_joint_angle_rad_vec[j];   // joint default
      std::cerr << "[joint default: i="<<i<<" default_angle_rad_ = "<<default_angle_rad_[i]<<"\n";
    } else {
      default_angle_rad_[i] = 0.f;
      std::cerr << "[policy] missing joint default: i="<<i<<"\n";
    }
  }
}

void GridRobot::motor2joint_mapping(){ //dataBusRead_的核心
  for (int i = 0; i < (int)active_.size(); ++i)
  {
    jointPos_rad_(i) = motorPos_rad_(i);
    jointVel_rad_(i) = motorVel_rad_(i);
    jointTau_Nm_(i) = motorTau_Nm_(i);
  }

  // 先默认 joint = motor
  for (int i = 0; i < (int)active_.size(); ++i)
  {
    jointPos_rad_(i) = motorPos_rad_(i);
    jointVel_rad_(i) = motorVel_rad_(i);
    jointTau_Nm_(i) = motorTau_Nm_(i);
  }

  // 只翻左膝 ,不再反转了
  // flipSignAt(jidx_L_knee_);
  // 右膝保持一致 => 不动

  // Ankle Fwd Kine Calc (Use Lambda Func)
  auto mapAnkleFK = [&](const AnkleIdx& A, const std::string& side){
    if (!A.ok()) return;
    if (A.m1<0 || A.m2<0 || A.jp<0 || A.jr<0) return;

    double m1_deg = ankle::rad2deg(motorPos_rad_(A.m1));
    double m2_deg = ankle::rad2deg(motorPos_rad_(A.m2));
    auto fk = ankle::forwardKinematics(ankleP_, m1_deg, m2_deg, 0.0, 0.0, 80, 1e-10);
    if (!fk.ok) return;

    // log
    if (side=="L") { last_L_pitch_deg_=fk.joint_deg.pitch_deg; last_L_roll_deg_=fk.joint_deg.roll_deg; }
    else           { last_R_pitch_deg_=fk.joint_deg.pitch_deg; last_R_roll_deg_=fk.joint_deg.roll_deg; }

    // joint pos
    jointPos_rad_(A.jp) = ankle::deg2rad(fk.joint_deg.pitch_deg);
    jointPos_rad_(A.jr) = ankle::deg2rad(fk.joint_deg.roll_deg);

    // joint vel
    Eigen::Vector2d qdot = ankle::fkVelocity_joint(
      ankleP_,
      jointPos_rad_(A.jp), jointPos_rad_(A.jr),
      motorPos_rad_(A.m1), motorPos_rad_(A.m2),
      motorVel_rad_(A.m1), motorVel_rad_(A.m2)
    );
    jointVel_rad_(A.jp) = qdot(0);
    jointVel_rad_(A.jr) = qdot(1);
    jointTau_Nm_(A.jp)  = 0.0;
    jointTau_Nm_(A.jr)  = 0.0;
    // joint Tau 不再需要计算，obs里面没有这个观测量，需要进一步确认
  };

  mapAnkleFK(ankleL_, "L");
  mapAnkleFK(ankleR_, "R");
  // Exec 刷新databus的jointState
  for(int i = 0; i<(int)active_.size(); ++i){
    databus_.jointState(i).q_joint   = (float)jointPos_rad_(i);
    databus_.jointState(i).dq_joint  = (float)jointVel_rad_(i);
    databus_.jointState(i).tau_joint = (float)jointTau_Nm_(i);
  }
}

void GridRobot::joint2motor_mapping()
{ // dataBusWrite_的核心

  auto mapAnkleIK = [&](const AnkleIdx& A){
    if (!A.ok()) return;
    if (A.m1<0 || A.m2<0 || A.jp<0 || A.jr<0) return;

    // Copy 一份 AnkleJoint的Cmd出来

    MotorCmd pitch_cmd_tmp = databus_.motorCmd(A.jp);
    MotorCmd roll_cmd_tmp  = databus_.motorCmd(A.jr);

    const bool enabled = 
      (pitch_cmd_tmp.enable && pitch_cmd_tmp.mode == CtrlMode::MOTION) ||
      (roll_cmd_tmp.enable  && roll_cmd_tmp.mode  == CtrlMode::MOTION);
    if(!enabled){
      databus_.motorCmd(A.m1) = MotorCmd{};
      databus_.motorCmd(A.m2) = MotorCmd{};
      return;
    }

    const double pitch_des = (double)pitch_cmd_tmp.q_des;
    const double roll_des  = (double)roll_cmd_tmp.q_des;
    const double pitch_d_des = (double)pitch_cmd_tmp.dq_des;
    const double roll_d_des  = (double)roll_cmd_tmp.dq_des;

    auto ik = ankle::inverseKinematics(
      ankleP_,
      ankle::rad2deg(pitch_des),
      ankle::rad2deg(roll_des)
    );
    
    if (!ik.ok) {
      databus_.motorCmd(A.m1) = MotorCmd{};
      databus_.motorCmd(A.m2) = MotorCmd{};
      return;
    }

    const auto& sol = ik.sol_minus;

    const double m1_des = ankle::deg2rad(sol.m1_deg);
    const double m2_des = ankle::deg2rad(sol.m2_deg);

    Eigen::Vector2d mdot = ankle::ikVelocity_motor(
      ankleP_,
      jointPos_rad_(A.jp), jointPos_rad_(A.jr),
      motorPos_rad_(A.m1), motorPos_rad_(A.m2),
      pitch_d_des, roll_d_des
    ); //使用当前的joint和motor位置信息构建jacobian，利用期望的pitch和roll速度来求出L和R踝关节的期望速度。

    // 直接填充MotorCmd

    MotorCmd c1{};
    c1.enable = true; c1.mode = CtrlMode::MOTION;
    c1.tau_ff = 0.f;
    c1.q_des  = (float)m1_des;
    c1.dq_des = (float)mdot(0);
    c1.kp     = kp_by_idx_[A.m1];
    c1.kd     = kd_by_idx_[A.m1];

    MotorCmd c2{};
    c2.enable = true; c2.mode = CtrlMode::MOTION;
    c2.tau_ff = 0.f;
    c2.q_des  = (float)m2_des;
    c2.dq_des = (float)mdot(1);
    c2.kp     = kp_by_idx_[A.m2];
    c2.kd     = kd_by_idx_[A.m2];

    databus_.motorCmd(A.m1) = c1;
    databus_.motorCmd(A.m2) = c2;
  };

  // 3) Exec
  mapAnkleIK(ankleL_);
  mapAnkleIK(ankleR_);
}

void GridRobot::cheatHoldDebugStep_(const TimeInfo &tm)
{
  clearAllMotorCmd_();

  // pre: "joint semantic" hold
  writeHoldCmd_(tm);

  const int n = (int)active_.size();
  std::vector<MotorCmd> pre(n);
  for (int i=0;i<n;++i) pre[i] = databus_.motorCmd(i);

  // post: mapped to motors (knee flip + ankle IK)
  joint2motor_mapping();

  std::vector<MotorCmd> post(n);
  for (int i=0;i<n;++i) post[i] = databus_.motorCmd(i);

  finalizeCmd();

  // print at telemetry rate
  static int divc = 0;
  int div = std::max(1, args_.rate_hz / std::max(1, args_.telemetry_hz));
  if (++divc < div) return;
  divc = 0;

  std::cout << "\n[CHEAT-RUN][hold] pre(joint q_des) -> post(motor q_des)\n";
  if (ankleL_.ok()) {
    std::cout << "  L_ankle joint(pitch,roll)=("
              << pre[ankleL_.jp].q_des << "," << pre[ankleL_.jr].q_des << ")  -> motor(m1,m2)=("
              << post[ankleL_.m1].q_des << "," << post[ankleL_.m2].q_des << ")\n";
  }
  if (ankleR_.ok()) {
    std::cout << "  R_ankle joint(pitch,roll)=("
              << pre[ankleR_.jp].q_des << "," << pre[ankleR_.jr].q_des << ")  -> motor(m1,m2)=("
              << post[ankleR_.m1].q_des << "," << post[ankleR_.m2].q_des << ")\n";
  }
  // if (jidx_L_knee_ >= 0 && jidx_L_knee_ < n) {
  //   std::cout << "  L_knee pre(joint)=" << pre[jidx_L_knee_].q_des
  //             << " -> post(motor)=" << post[jidx_L_knee_].q_des << "  (knee flip)\n";
  // }
}

void GridRobot::cheatPlantStep_(double t_sec)
{
  const int n = (int)active_.size();
  const double dt = std::max(1e-6, dt_us_ * 1e-6);

  // ramp finished? -> snap-to-target
  bool snap = false;
  if ((args_.test == "hold" || args_.test == "policy") && hold_ramp_t0_sec_ >= 0.0) {
    snap = (t_sec - hold_ramp_t0_sec_) >= hold_ramp_dur_sec_;
  }

  for (int i=0;i<n;++i) {
    auto &c = databus_.motorCmd(i);

    // default: no motion
    motorVel_rad_(i) = 0.0;
    motorTau_Nm_(i)  = 0.0;

    if (!c.enable || c.mode == CtrlMode::DISABLE) continue;

    // only simulate position tracking when kp>0 (damping kp=0 will not move)
    if (c.mode == CtrlMode::MOTION && c.kp > 1e-6f) {
      const double q0 = motorPos_rad_(i);
      double q1 = q0;

      if (snap) {
        // the behavior you asked: after last hold cmd -> all reached
        q1 = (double)c.q_des;
      } else {
        // simple 1st-order tracking during ramp
        const double tau = 0.08; // 80ms time constant, tweak if you want
        const double alpha = std::min(1.0, dt / tau);
        q1 = q0 + alpha * ((double)c.q_des - q0);
      }

      motorPos_rad_(i) = q1;
      motorVel_rad_(i) = (q1 - q0) / dt;
      motorTau_Nm_(i)  = (double)c.tau_ff;
    }
    else if (c.mode == CtrlMode::TORQUE) {
      motorTau_Nm_(i) = (double)c.tau_ff;
    }
  }
}

void GridRobot::startPolicyIoLog_()
{
  stopPolicyIoLog_();

  policy_io_log_path_ = log_dir_ + "/policy_io_first10s.csv";
  try {
    std::filesystem::create_directories(log_dir_);
  } catch (const std::exception& e) {
    std::cerr << "[policy_io_log] create_directories failed: " << e.what() << "\n";
    return;
  }

  fpolicy_io_ = std::fopen(policy_io_log_path_.c_str(), "w");
  if (!fpolicy_io_) {
    std::perror("[policy_io_log] fopen failed");
    return;
  }
  setvbuf(fpolicy_io_, nullptr, _IOLBF, 0);

  const int fs = std::max(1, args_.frame_stack);
  const int obs_dim = fs * kNumSingleObs;

  // header
  std::fprintf(fpolicy_io_, "t,lowlevel_cnt,policy_run_steps,warmup");
  for (int i = 0; i < obs_dim; ++i) std::fprintf(fpolicy_io_, ",obs%d", i);
  for (int i = 0; i < kNumActions; ++i) std::fprintf(fpolicy_io_, ",a%d", i);
  std::fprintf(fpolicy_io_, "\n");

  policy_io_rows_ = 0;
  policy_io_log_active_ = true;

  std::cout << "[policy_io_log] => " << policy_io_log_path_
            << " (first " << policy_io_log_seconds_ << "s, obs_dim=" << obs_dim
            << ", decimation=" << args_.decimation << ")\n";
}

void GridRobot::stopPolicyIoLog_()
{
  policy_io_log_active_ = false;
  if (fpolicy_io_) {
    std::fflush(fpolicy_io_);
    std::fclose(fpolicy_io_);
    fpolicy_io_ = nullptr;
  }
}

void GridRobot::logPolicyIoRow_(const TimeInfo& tm)
{
  if (!policy_io_log_active_ || !fpolicy_io_) return;

  const double t_rel = tm.policy_sec;
  if (t_rel > policy_io_log_seconds_) {
    std::cout << "[policy_io_log] done. rows=" << policy_io_rows_
              << " (t_rel=" << t_rel << ")\n";
    stopPolicyIoLog_();
    return;
  }

  const bool warmup = (policy_run_steps_ < std::max(0, args_.warmup_steps));

  std::fprintf(fpolicy_io_, "%.6f,%lld,%d,%d",
               t_rel,
               (long long)lowlevel_cnt_,
               policy_run_steps_,
               warmup ? 1 : 0);

  // obs = policy_input_ (stacked, exactly what is fed to torch)
  // policy_input_ size should be fs*kNumSingleObs
  for (size_t i = 0; i < policy_input_.size(); ++i) {
    std::fprintf(fpolicy_io_, ",%.8f", (double)policy_input_[i]);
  }

  // action_ = clipped raw policy output (NOT warmup-forced)
  for (int i = 0; i < kNumActions; ++i) {
    std::fprintf(fpolicy_io_, ",%.8f", (double)action_[i]);
  }
  std::fprintf(fpolicy_io_, "\n");

  if ((++policy_io_rows_ % 200) == 0) std::fflush(fpolicy_io_);
}
