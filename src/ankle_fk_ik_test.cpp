#include "ankle_kinematics.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

static void run_one(const ankle::AnkleParams& P,
                    double pitch_in, double roll_in,
                    bool use_radians)
{
  const double pitch_deg = use_radians ? ankle::rad2deg(pitch_in) : pitch_in;
  const double roll_deg  = use_radians ? ankle::rad2deg(roll_in)  : roll_in;

  std::cout << "\n==============================\n";
  std::cout << "Ankle: " << P.name
            << " (invert m1=" << P.invert_motor1
            << ", m2=" << P.invert_motor2 << ")\n";
  std::cout << "Target joint: pitch=" << pitch_in << (use_radians ? " rad" : " deg")
            << ", roll=" << roll_in << (use_radians ? " rad" : " deg")
            << "  =>  (" << pitch_deg << " deg, " << roll_deg << " deg)\n";

  // IK: joint(deg) -> motor(deg), two branches
  auto ik = ankle::inverseKinematics(P, pitch_deg, roll_deg);
  std::cout << "IK ok=" << ik.ok
            << " rho1=" << ik.rho1 << " rho2=" << ik.rho2 << "\n";

  auto test_branch = [&](const char* tag, const ankle::MotorAnglesDeg& sol){
    auto fk = ankle::forwardKinematics(P, sol.m1_deg, sol.m2_deg, 0.0, 0.0, 80, 1e-12);

    std::cout << "\n[" << tag << "] motor(deg): m1=" << sol.m1_deg << ", m2=" << sol.m2_deg << "\n";
    std::cout << "FK ok=" << fk.ok << " iters=" << fk.iters
              << " residual=" << std::scientific << std::setprecision(6) << fk.final_residual
              << std::defaultfloat << "\n";
    std::cout << "FK joint(deg): pitch=" << fk.joint_deg.pitch_deg
              << " roll=" << fk.joint_deg.roll_deg << "\n";

    const double ep = fk.joint_deg.pitch_deg - pitch_deg;
    const double er = fk.joint_deg.roll_deg  - roll_deg;
    std::cout << "Error(deg): dp=" << ep << " dr=" << er
              << " |err|=" << std::sqrt(ep*ep + er*er) << "\n";
  };

  // 两支解都回代 FK 验证
  test_branch("minus/minus", ik.sol_minus);
  test_branch("plus/plus",   ik.sol_plus);
}

int main()
{
  // 1) 基础几何参数（你项目里已有实现）
  ankle::AnkleParams base = ankle::make_params_v2();

  // 2) 左右踝：通常需要根据装配方向设置 invert_motor1/2
  ankle::AnkleParams L = base;
  L.name = "LEFT_ANKLE";
  // TODO: 如果你左脚电机方向与几何坐标系相反，就改这里
  L.invert_motor1 = true;
  L.invert_motor2 = false;

  ankle::AnkleParams R = base;
  R.name = "RIGHT_ANKLE";
  // TODO: 如果你右脚电机方向与几何坐标系相反，就改这里
  R.invert_motor1 = true;
  R.invert_motor2 = false;

  // 3) 期望值（按 rad 写）
  const double pitch_target = -0.4; // rad
  const double roll_target  =  0.1; // rad
  const bool use_radians = true;    // 你如果就是 deg，把它改成 false

  run_one(L, pitch_target, roll_target, use_radians);
  run_one(R, pitch_target, roll_target, use_radians);

  return 0;
}
