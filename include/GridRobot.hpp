#pragma once
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>
#include <torch/script.h>
#include <deque>
#include <array>
#include "ankle_kinematics.hpp"
#include "config.hpp"
#include "databus.hpp"
#include "motor.hpp"
#include "router.hpp"
#include "stats.hpp"
#include "usb2can_transport.hpp"
#include "witmotion_imu.hpp"
#include "util.hpp"
#include <poll.h>
#include <unistd.h>
#include <filesystem>
#include <iomanip>

struct AnkleIdx {
  int m1=-1, m2=-1;      // ankle motors (L/R)
  int jp=-1, jr=-1;      // ankle joints (pitch/roll)
  bool ok() const { return m1>=0 && m2>=0 && jp>=0 && jr>=0; }
};
class GridRobot {
public:
  struct Args {
    std::string config = "../config/config.yaml";
    std::string only_group_csv;      // e.g. "u1b1,u2b2"
    bool skip_zero_sta = true;  // 跳过 set_zero_sta
    bool skip_set_zero = false;  // 跳过 set_zero
    bool verify_zero_on_boot = true; // 跳过时是否打印/简单校验
    int rate_hz = 1000;
    std::string test = "damping";    // damping / sine / hold / fkik (dry-run)
    float sine_amp = 0.3f;           // rad (databus sine), fkik会内部转deg
    float sine_freq = 0.5f;          // Hz
    float kp = 40.f;
    float kd = 2.0f;
    bool dry_run = false;
    // std::string imu_port = "/dev/ttyUSB0";
    std::string imu_port = "/dev/imu";

    int imu_baud = 921600;
    bool imu_enable = true;
    bool imu_only = false;
    bool cheat_run = false; // cheatRun: fake sensor states using YAML defaults, no hardware IO
    int telemetry_hz = 1;
    std::string policy_path = "/home/jhuo/rl_learning/humanoid-gym/logs/DreamBot_ppo/exported/policies/policy_1.pt";
    int frame_stack = 15;
    int decimation = 10;
    float action_scale = 0.25f; // 对齐 cfg.control.action_scale
    float clip_actions = 18.0f;
    float clip_obs = 18.0f;
    float gait_period = 0.64f;

    // cmd
    float cmd_vx = 0.4f;
    float cmd_vy = 0.0f;
    float cmd_dyaw = 0.0f;

    // obs scales（对齐 cfg.normalization.obs_scales）
    float obs_lin_vel_scale = 2.0f;
    float obs_ang_vel_scale = 1.0f;
    float obs_dof_pos_scale = 1.0f;
    float obs_dof_vel_scale = 0.05f;

    // warmup
    int warmup_steps = 800; // 对齐 python count_lowlevel>800
  };

  static Args parse_args(int argc, char** argv);

  GridRobot() = default;
  ~GridRobot();

  bool init(const Args& args);
  void exec(std::atomic<bool>& run_flag);

private:
  struct TimeInfo
  {
    double program_sec = 0.0; // since control loop start
    double policy_sec = 0.0;  // since POLICY RUN enter (gate==2), else 0
  };
  // ---- helpers ----
  static bool group_selected(const std::string &group, const std::vector<std::string> &only_groups);
  bool initDryRun();
  bool initHardware();
  bool initCheatRun_();
  void execCheatRun_(std::atomic<bool> &run_flag);
  void cheatPlantStep_(double t_sec);
  void cheatRunFunc_(); // fake motor/joint/imu states to defaults
  void execDryRun(std::atomic<bool> &run_flag);
  void execHardware(std::atomic<bool> &run_flag);
  void buildIdxOnce_();      // motorName -> index（active顺序）
  void buildJointIdxOnce_(); // jointName -> index（active顺序）
  void openLog_();
  void closeLog_();
  void powerOnSequence_();
  void stopAll_();
  void databusRead_();
  void applyCmd_();
  void printStep_();
  void checkLimit_();
  void writeAllDamping_();
  void databusWrite_(const TimeInfo &tm);
  void logStep_(const TimeInfo &tm);
  void policyStep_(const TimeInfo &tm);

  void cheatHoldDebugStep_(const TimeInfo &tm);

  void motor2joint_mapping();
  void joint2motor_mapping();

  void initPolicyJointMapAndDefaults_();
  void initAnkleIdxOnce_();

  void clearAllMotorCmd_();
  void finalizeCmd();

  void writeDampingCmd_();
  void writeHoldCmd_(const TimeInfo &tm);
  void writePolicyCmd_(const TimeInfo &tm);

private:
  Args args_{};
  std::vector<std::string> only_groups_;
  AppConfig cfg_{};

  ankle::AnkleParams ankleP_{};
  AnkleIdx ankleL_, ankleR_;
  
  inline bool isAnkleMotor_(int i) const {
    return i==ankleL_.m1 || i==ankleL_.m2 || i==ankleR_.m1 || i==ankleR_.m2;
  }
  //  ---- IMU ----
  WitMotionImuRunner imu_runner_;
  // ---- DRY-RUN state ----
  struct Sel { MotorKey mk; int sidx; };
  std::vector<Sel> dry_selected_;
  std::unordered_map<std::string,int> dry_name2dummy_;

  // ---- Hardware state ----
  std::vector<bool> need_dev_;
  std::vector<std::unique_ptr<Usb2CanDevice>> dev_objs_;
  std::vector<Usb2CanDevice*> dev_ptrs_;   // aligned by cfg.devices index
  std::unique_ptr<Usb2CanRouter> router_;

  Stats stats_;
  std::vector<std::unique_ptr<RobStrideMotor>> motors_;
  std::vector<RobStrideMotor*> active_;

  std::vector<float> kp_by_idx_;
  std::vector<float> kd_by_idx_;
  std::vector<float> default_motor_angle_rad_vec; // 对应 YAML: default_motor_angle_rad
  std::vector<float> default_joint_angle_rad_vec; // 对应 YAML: default_joint_angle_rad

  std::unordered_map<std::string,int> motor_idx_; // motorName -> active index

  DataBus databus_{0};

  // ---- logging ----
  int log_every_ = 5;
  std::string log_dir_  = "/home/jhuo/robstride_usb2can_ctrl/logs";
  std::string log_path_ = "/home/jhuo/robstride_usb2can_ctrl/logs/track.csv";
  FILE* flog_ = nullptr;

  // ---- loop timing ----
  int dt_us_ = 1000;
  std::chrono::steady_clock::time_point t0_program_; // program start (loop start)
  std::chrono::steady_clock::time_point next_;

  // ---- unified time ----
  TimeInfo tm_{};
  double policy_t0_program_sec_ = -1.0; // set when entering POLICY RUN (gate==2)

  inline void resetProgramTime_(const std::chrono::steady_clock::time_point &now)
  {
    t0_program_ = now;
    tm_ = TimeInfo{};
    policy_t0_program_sec_ = -1.0;
  }

  inline void updateTime_(const std::chrono::steady_clock::time_point &now)
  {
    tm_.program_sec = std::chrono::duration<double>(now - t0_program_).count();
    tm_.policy_sec = (policy_t0_program_sec_ >= 0.0)
                         ? (tm_.program_sec - policy_t0_program_sec_)
                         : 0.0;
  }

  inline const TimeInfo &time_() const { return tm_; }

  // ---- scratch ----
  Eigen::VectorXd motorPos_rad_;
  Eigen::VectorXd motorVel_rad_;
  Eigen::VectorXd motorTau_Nm_;
  Eigen::VectorXd motorCmdPos_rad_;
  Eigen::VectorXd motorCmdVel_rad_;

  // joint 命名与索引
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, int> joint_idx_;
  int jidx_L_knee_ = -1;   // joint index in jointPos_rad_
  int jidx_R_knee_ = -1;


  // joint 缓存
  Eigen::VectorXd jointPos_rad_;
  Eigen::VectorXd jointVel_rad_;
  Eigen::VectorXd jointTau_Nm_;
  Eigen::VectorXd jointCmdPos_rad_;
  Eigen::VectorXd jointCmdVel_rad_;

  // 数值 Jacobian 缓存
  Eigen::Matrix2d J_lankle_deg_ = Eigen::Matrix2d::Identity();
  int jac_div_ = 0;
  int jac_every_ = 10; // 每 jac_every_ 个周期更新一次 J（500Hz 时=50Hz）

  // ---- ankle telemetry ----
  // Left ankle
  double last_L_pitch_deg_ = NAN;
  double last_L_roll_deg_ = NAN;
  double last_L_fkik_em1_deg_ = NAN;
  double last_L_fkik_em2_deg_ = NAN;

  // Right ankle
  double last_R_pitch_deg_ = NAN;
  double last_R_roll_deg_ = NAN;
  double last_R_fkik_em1_deg_ = NAN;
  double last_R_fkik_em2_deg_ = NAN;

  static constexpr int kNumActions = 12;
  static constexpr int kNumSingleObs = 47;

  torch::jit::script::Module policy_;
  bool policy_ok_ = false;

  int lowlevel_cnt_ = 0; // 类似 python count_lowlevel
  std::array<float, kNumActions> last_action_{};

  std::deque<std::array<float, kNumSingleObs>> hist_obs_;
  std::vector<float> policy_input_; // size = frame_stack*47

  // action 输出缓存（每次推理更新，否则保持）
  std::array<float, kNumActions> action_{};

  struct Limit
  {
    float lo, hi;
  };
  std::array<Limit, kNumActions> joint_limit_{{ //应该改为joint位置
      // Left1..6
      {-0.5f, 0.8f},
      {-0.3f, 0.3f},
      // {-0.3f, 0.1f},
      {-0.9f, 0.9f},
      // {-0.05f, 1.58f}, //在urdf 0位姿态下左膝电机默认方向与urdf方向相反。default joint angle = 0.8 而电机位置为-0.8 需要处理
      {-1.58f, 0.05f}, //在urdf 0位姿态下左膝电机默认方向与urdf方向相同了。default joint angle =-0.45
      // {-0.1f, 0.5f},
      // {-0.5f, 0.1f},
      {-0.5f, 0.1f},
      {-0.52f, 0.52f},
      // Right1..6
      {-0.8f, 0.5f},
      // {-0.1f, 0.3f},
      {-0.3f, 0.3f},
      {-0.9f, 0.9f},
      {-0.05f, 1.58f},
      {-0.5f, 0.1f},
      {-0.52f, 0.52f},
  }};
  std::array<float, kNumActions> default_angle_rad_{};
  std::array<int,  kNumActions> policy_joint_idx_{};
  std::atomic<bool> emergency_damping_{false};
  int emergency_idx_ = -1;
  float emergency_val_ = 0.f;

  int  policy_gate_state_ = 0;   // 0: HOLD(default)  1: WAIT(enter)  2: RUN(policy)
  bool policy_gate_prompted_ = false;
  int  policy_run_steps_ = 0;    // RUN阶段计数，用于warmup


  double hold_ramp_t0_sec_ = -1.0;
  double hold_ramp_dur_sec_ = 2.0;  // 2秒拉到最终hold姿态

// ---- policy io logging (obs + action, first 10s from policy RUN start) ----
FILE* fpolicy_io_ = nullptr;
std::string policy_io_log_path_;
bool policy_io_log_active_ = false;
double policy_io_log_seconds_ = 10.0;
int64_t policy_io_rows_ = 0;

void startPolicyIoLog_();   // called at policy RUN enter (warmup start)
void stopPolicyIoLog_();
void logPolicyIoRow_(const TimeInfo& tm); // called once per inference (decimated) after action_ updated

};
