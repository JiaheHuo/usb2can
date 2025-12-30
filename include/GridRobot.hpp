#pragma once
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "ankle_kinematics.hpp"
#include "config.hpp"
#include "databus.hpp"
#include "motor.hpp"
#include "router.hpp"
#include "stats.hpp"
#include "usb2can_transport.hpp"
#include "witmotion_imu.hpp"
#include "util.hpp"

class GridRobot {
public:
  struct Args {
    std::string config = "../config/motors.yaml";
    std::string only_group_csv;      // e.g. "u1b1,u2b2"
    int rate_hz = 500;
    std::string test = "damping";    // damping / sine / hold / fkik (dry-run)
    float sine_amp = 0.3f;           // rad (databus sine), fkik会内部转deg
    float sine_freq = 0.5f;          // Hz
    float kp = 40.f;
    float kd = 2.0f;
    bool dry_run = false;
    std::string imu_port = "/dev/ttyUSB0";
    int imu_baud = 921600;
    bool imu_enable = true;
    bool imu_only = false;
    int telemetry_hz = 2;  
  };

  static Args parse_args(int argc, char** argv);

  GridRobot() = default;
  ~GridRobot();

  bool init(const Args& args);
  void exec(std::atomic<bool>& run_flag);

private:
  // ---- helpers ----
  static bool group_selected(const std::string& group, const std::vector<std::string>& only_groups);

  bool initDryRun();
  bool initHardware();

  void execDryRun(std::atomic<bool>& run_flag);
  void execHardware(std::atomic<bool>& run_flag);

  void buildIdxOnce_();      // motorName -> index（active顺序）
  void buildJointIdxOnce_();      // jointName -> index（active顺序）
  void openLog_();
  void closeLog_();
  void powerOnSequence_();
  void stopAll_();

  // loop steps: read -> ankle calc -> write -> apply -> log/print
  void databusRead_();
  void ankleCalc_();         // FK/IK
  void databusWrite_(double t);
  void applyCmd_();
  void logStep_(double t);
  void printStep_();

private:
  Args args_{};
  std::vector<std::string> only_groups_;
  AppConfig cfg_{};

  ankle::AnkleParams ankleP_{};
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

  std::unordered_map<std::string,int> motor_idx_; // motorName -> active index

  DataBus databus_{0};

  // ---- logging ----
  int log_every_ = 5;
  std::string log_dir_  = "/home/jhuo/robstride_usb2can_ctrl/logs";
  std::string log_path_ = "/home/jhuo/robstride_usb2can_ctrl/logs/track.csv";
  FILE* flog_ = nullptr;

  // ---- loop timing ----
  int dt_us_ = 2000;
  std::chrono::steady_clock::time_point t0_;
  std::chrono::steady_clock::time_point next_;

  // ---- scratch ----
  Eigen::VectorXd motorPos_rad_;
  Eigen::VectorXd motorVel_rad_;
  Eigen::VectorXd motorTau_Nm_;
  Eigen::VectorXd motorCmdPos_rad_;
  Eigen::VectorXd motorCmdVel_rad_;

  // joint 命名与索引
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, int> joint_idx_;

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
};
