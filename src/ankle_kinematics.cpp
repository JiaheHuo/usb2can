#include "ankle_kinematics.hpp"
#include <cmath>
#include <iostream>

namespace ankle {

static constexpr double PI = 3.14159265358979323846;

double deg2rad(double d) { return d * PI / 180.0; }
double rad2deg(double r) { return r * 180.0 / PI; }

double wrapToPi(double x) {
  x = std::fmod(x + PI, 2.0 * PI);
  if (x < 0) x += 2.0 * PI;
  return x - PI;
}

Matrix3d Rx(double b) {
  const double c = std::cos(b), s = std::sin(b);
  Matrix3d R;
  R << 1, 0, 0,
       0, c, -s,
       0, s,  c;
  return R;
}
Matrix3d Ry(double a) {
  const double c = std::cos(a), s = std::sin(a);
  Matrix3d R;
  R <<  c, 0, s,
        0, 1, 0,
       -s, 0, c;
  return R;
}
Matrix3d dRx(double b) {
  const double c = std::cos(b), s = std::sin(b);
  Matrix3d d;
  d << 0, 0, 0,
       0, -s, -c,
       0,  c, -s;
  return d;
}
Matrix3d dRy(double a) {
  const double c = std::cos(a), s = std::sin(a);
  Matrix3d d;
  d << -s, 0,  c,
        0, 0,  0,
       -c, 0, -s;
  return d;
}

AnkleParams make_params_v2() {
  AnkleParams P;
  P.name = "ANKLE";
  const double deg13 = deg2rad(13.0);
  const double Cx = 75.0 * std::cos(deg13);
  const double Cz = 75.0 * std::sin(deg13);
  const double Cy = 21.0;
  const double Width = 133.0;

  // A1 对应左侧（motor1）
  P.A1 = Vector3d(0.0,  Width/2.0, -80.0);          // in {O}
  P.B1 = Vector3d(-Cx,  Cy,        -80.0 - Cz);     // in {O}
  P.C1 = Vector3d(-Cx,  Cy,        -Cz);            // in {Ow}
  P.Len_Long = 228.0;

  // A2 对应右侧（motor2）
  P.A2 = Vector3d(0.0, -Width/2.0, -80.0);          // in {O}
  P.B2 = Vector3d(-Cx, -Cy,        -80.0 - Cz);     // in {O}
  P.C2 = Vector3d(-Cx, -Cy,        -Cz);            // in {Ow}
  P.Len_Short = 228.0;

  // 万向节中心 in {O}
  P.Ow = Vector3d(0.0, 0.0, -308.0);

  P.invert_motor1 = true;
  P.invert_motor2 = false;

  return P;
}


FwdOut2D computeFwd(const AnkleParams& P, double alpha, double beta,
                    double th1_link, double th2_link) {
  FwdOut2D out;

  const Matrix3d R_ab = Ry(alpha) * Rx(beta);

  // --- 1st bar (Len_Long) ---
  const Vector3d p1 = P.A1 - (R_ab * P.C1 + P.Ow);
  const Vector3d v1 = P.B1 - P.A1;
  const Vector3d u1 = p1 + Ry(th1_link) * v1;
  out.f1 = u1.squaredNorm() - P.Len_Long * P.Len_Long;

  // --- 2nd bar (Len_Short) ---
  const Vector3d p2 = P.A2 - (R_ab * P.C2 + P.Ow);
  const Vector3d v2 = P.B2 - P.A2;
  const Vector3d u2 = p2 + Ry(th2_link) * v2;
  out.f2 = u2.squaredNorm() - P.Len_Short * P.Len_Short;

  // derivatives wrt alpha,beta
  const Vector3d dp1_dalpha = -(dRy(alpha) * Rx(beta) * P.C1);
  const Vector3d dp1_dbeta  = -(Ry(alpha)  * dRx(beta) * P.C1);
  const double df1_dalpha = 2.0 * u1.dot(dp1_dalpha);
  const double df1_dbeta  = 2.0 * u1.dot(dp1_dbeta);

  const Vector3d dp2_dalpha = -(dRy(alpha) * Rx(beta) * P.C2);
  const Vector3d dp2_dbeta  = -(Ry(alpha)  * dRx(beta) * P.C2);
  const double df2_dalpha = 2.0 * u2.dot(dp2_dalpha);
  const double df2_dbeta  = 2.0 * u2.dot(dp2_dbeta);

  out.dF(0,0) = df1_dalpha; out.dF(0,1) = df1_dbeta;
  out.dF(1,0) = df2_dalpha; out.dF(1,1) = df2_dbeta;

  return out;
}

static inline double motorToLink(double m_rad, bool inv) {
  return inv ? -m_rad : m_rad;
}
static inline double linkToMotor(double th_link_rad, bool inv) {
  return inv ? -th_link_rad : th_link_rad;
}

FKReport forwardKinematics(const AnkleParams& P, double motor1_deg, double motor2_deg,
                           double alpha0_rad, double beta0_rad,
                           int max_iter, double tol) {
  FKReport rep;

  // motor(deg)->rad
  const double motor1 = deg2rad(motor1_deg);
  const double motor2 = deg2rad(motor2_deg);

  // motor coord -> link coord
  const double th1 = motorToLink(motor1, P.invert_motor1);
  const double th2 = motorToLink(motor2, P.invert_motor2);

  double a = alpha0_rad;
  double b = beta0_rad;

  auto residual = [&](double aa, double bb) {
    const auto v = computeFwd(P, aa, bb, th1, th2);
    return std::sqrt(v.f1*v.f1 + v.f2*v.f2);
  };

  double res0 = residual(a,b);

  for (int it = 0; it < max_iter; ++it) {
    const auto val = computeFwd(P, a, b, th1, th2);
    const double res = std::sqrt(val.f1*val.f1 + val.f2*val.f2);

    rep.iters = it;
    rep.final_residual = res;

    if (res < tol) {
      rep.ok = true;
      break;
    }

    // Newton step: [a,b] -= J^{-1} f
    const Vector2d fvec(val.f1, val.f2);

    // 若矩阵奇异，退出
    const double det = val.dF.determinant();
    if (std::fabs(det) < 1e-14) {
      rep.ok = false;
      break;
    }

    Vector2d delta = val.dF.inverse() * fvec;

    // 简单线搜索，避免发散
    double step = 1.0;
    bool accepted = false;
    for (int ls = 0; ls < 12; ++ls) {
      const double na = wrapToPi(a - step * delta(0));
      const double nb = wrapToPi(b - step * delta(1));
      const double nres = residual(na, nb);
      if (nres <= res) {
        a = na; b = nb;
        accepted = true;
        break;
      }
      step *= 0.5;
    }
    if (!accepted) {
      // 接受不了任何步长，认为失败
      rep.ok = false;
      break;
    }

    res0 = res;
  }

  rep.joint_deg.pitch_deg = rad2deg(a);
  rep.joint_deg.roll_deg  = rad2deg(b);
  return rep;
}

static bool ik_one_motor(const Vector3d& A, const Vector3d& B, const Vector3d& C,
                         const Vector3d& Ow, double L,
                         double alpha, double beta,
                         double& th_minus, double& th_plus, double& rho_out) {
  const Matrix3d R_ab = Ry(alpha) * Rx(beta);
  const Vector3d p = A - (R_ab * C + Ow);
  const Vector3d v = B - A;

  const double px = p(0), py = p(1), pz = p(2);
  const double vx = v(0), vy = v(1), vz = v(2);

  const double varA = (px*px + pz*pz) + (vx*vx + vz*vz) + ((py+vy)*(py+vy));
  const double varB = 2.0 * (px*vx + pz*vz);
  const double varC = 2.0 * (px*vz - pz*vx);

  const double varR = std::sqrt(varB*varB + varC*varC);
  if (varR < 1e-16) return false;

  const double phi = std::atan2(varC, varB);
  const double rho = (L*L - varA) / varR;
  rho_out = rho;

  if (rho < -1.0 || rho > 1.0) return false;

  const double c = std::acos(rho);
  th_minus = phi - c;
  th_plus  = phi + c;
  return true;
}

IKReport inverseKinematics(const AnkleParams& P, double alpha_deg, double beta_deg) {
  IKReport rep;

  const double alpha = deg2rad(alpha_deg);
  const double beta  = deg2rad(beta_deg);

  double th1_minus=0, th1_plus=0, rho1=0;
  double th2_minus=0, th2_plus=0, rho2=0;

  const bool ok1 = ik_one_motor(P.A1, P.B1, P.C1, P.Ow, P.Len_Long,
                               alpha, beta, th1_minus, th1_plus, rho1);
  const bool ok2 = ik_one_motor(P.A2, P.B2, P.C2, P.Ow, P.Len_Short,
                               alpha, beta, th2_minus, th2_plus, rho2);

  rep.rho1 = rho1;
  rep.rho2 = rho2;
  rep.ok = ok1 && ok2;
  if (!rep.ok) return rep;

  // link coord -> motor coord
  const double m1_minus = linkToMotor(th1_minus, P.invert_motor1);
  const double m2_minus = linkToMotor(th2_minus, P.invert_motor2);
  const double m1_plus  = linkToMotor(th1_plus,  P.invert_motor1);
  const double m2_plus  = linkToMotor(th2_plus,  P.invert_motor2);

  rep.sol_minus.m1_deg = rad2deg(m1_minus);
  rep.sol_minus.m2_deg = rad2deg(m2_minus);
  rep.sol_plus.m1_deg  = rad2deg(m1_plus);
  rep.sol_plus.m2_deg  = rad2deg(m2_plus);

  return rep;
}
Eigen::Matrix2d buildJacIK_link(const AnkleParams& P,
                                double alpha_rad, double beta_rad,
                                double th1_link_rad, double th2_link_rad)
{
  using Eigen::Matrix2d;
  using Eigen::Matrix3d;
  using Eigen::Vector3d;

  Matrix2d J = Matrix2d::Zero();

  // ---------- Row0: motor1/left bar ----------
  {
    const Matrix3d R_ab = Ry(alpha_rad) * Rx(beta_rad);
    const Vector3d p1 = P.A1 - (R_ab * P.C1 + P.Ow);
    const Vector3d v1 = P.B1 - P.A1;
    const Vector3d u1 = p1 + Ry(th1_link_rad) * v1;

    const double df_dth1 = 2.0 * u1.dot(dRy(th1_link_rad) * v1);

    const Vector3d dp1_dalpha = -(dRy(alpha_rad) * Rx(beta_rad) * P.C1);
    const Vector3d dp1_dbeta  = -(Ry(alpha_rad)  * dRx(beta_rad) * P.C1);

    const double df_dalpha = 2.0 * u1.dot(dp1_dalpha);
    const double df_dbeta  = 2.0 * u1.dot(dp1_dbeta);

    // dth/dq = - (df/dq) / (df/dth)
    const double dth_dalpha = - df_dalpha / df_dth1;
    const double dth_dbeta  = - df_dbeta  / df_dth1;

    J(0,0) = dth_dalpha;
    J(0,1) = dth_dbeta;
  }

  // ---------- Row1: motor2/right bar ----------
  {
    const Eigen::Matrix3d R_ab = Ry(alpha_rad) * Rx(beta_rad);
    const Eigen::Vector3d p2 = P.A2 - (R_ab * P.C2 + P.Ow);
    const Eigen::Vector3d v2 = P.B2 - P.A2;
    const Eigen::Vector3d u2 = p2 + Ry(th2_link_rad) * v2;

    const double df_dth2 = 2.0 * u2.dot(dRy(th2_link_rad) * v2);

    const Eigen::Vector3d dp2_dalpha = -(dRy(alpha_rad) * Rx(beta_rad) * P.C2);
    const Eigen::Vector3d dp2_dbeta  = -(Ry(alpha_rad)  * dRx(beta_rad) * P.C2);

    const double df_dalpha = 2.0 * u2.dot(dp2_dalpha);
    const double df_dbeta  = 2.0 * u2.dot(dp2_dbeta);

    const double dth_dalpha = - df_dalpha / df_dth2;
    const double dth_dbeta  = - df_dbeta  / df_dth2;

    J(1,0) = dth_dalpha;
    J(1,1) = dth_dbeta;
  }

  return J;
}

Eigen::Matrix2d buildJacIK_motor(const AnkleParams& P,
                                 double alpha_rad, double beta_rad,
                                 double m1_motor_rad, double m2_motor_rad)
{
  // motor -> link
  const double th1_link = motorToLink(m1_motor_rad, P.invert_motor1);
  const double th2_link = motorToLink(m2_motor_rad, P.invert_motor2);

  // J_link = d(theta_link)/d(q)
  Eigen::Matrix2d J_link = buildJacIK_link(P, alpha_rad, beta_rad, th1_link, th2_link);

  // th_link = s * m_motor, s= (invert? -1 : +1)
  // m_motor = s * th_link  (因为 s=±1)
  // => J_motor = dm/dq = s * dth/dq
  Eigen::Matrix2d S = Eigen::Matrix2d::Identity();
  S(0,0) = (P.invert_motor1 ? -1.0 : +1.0);
  S(1,1) = (P.invert_motor2 ? -1.0 : +1.0);

  return S * J_link;
}

// ---------------- Velocity IK: qdot -> mdot ----------------
Eigen::Vector2d ikVelocity_motor(const AnkleParams& P,
                                 double alpha_rad, double beta_rad,
                                 double m1_motor_rad, double m2_motor_rad,
                                 double alpha_dot_rad, double beta_dot_rad)
{
  const Eigen::Matrix2d Jm = buildJacIK_motor(P, alpha_rad, beta_rad, m1_motor_rad, m2_motor_rad);
  return Jm * Eigen::Vector2d(alpha_dot_rad, beta_dot_rad);
}

// ---------------- Velocity FK: mdot -> qdot ----------------
Eigen::Vector2d fkVelocity_joint(const AnkleParams& P,
                                 double alpha_rad, double beta_rad,
                                 double m1_motor_rad, double m2_motor_rad,
                                 double m1_dot_rad, double m2_dot_rad)
{
  const Eigen::Matrix2d Jm = buildJacIK_motor(P, alpha_rad, beta_rad, m1_motor_rad, m2_motor_rad);
  return Jm.inverse() * Eigen::Vector2d(m1_dot_rad, m2_dot_rad);
}

// ---------------- Torque IK: tau_q -> tau_motor ----------------
Eigen::Vector2d ikTorque_motor(const AnkleParams& P,
                               double alpha_rad, double beta_rad,
                               double m1_motor_rad, double m2_motor_rad,
                               double tau_alpha, double tau_beta,
                               double torque_sign)
{
  const Eigen::Matrix2d Jm = buildJacIK_motor(P, alpha_rad, beta_rad, m1_motor_rad, m2_motor_rad);
  // tau_motor = sign * (J^{-T}) * tau_joint
  return torque_sign * (Jm.transpose().inverse()) * Eigen::Vector2d(tau_alpha, tau_beta);
}

// ---------------- Torque FK: tau_motor -> tau_q ----------------
Eigen::Vector2d fkTorque_joint(const AnkleParams& P,
                               double alpha_rad, double beta_rad,
                               double m1_motor_rad, double m2_motor_rad,
                               double tau_m1, double tau_m2,
                               double torque_sign)
{
  const Eigen::Matrix2d Jm = buildJacIK_motor(P, alpha_rad, beta_rad, m1_motor_rad, m2_motor_rad);
  // tau_joint = sign * (J^T) * tau_motor
  return torque_sign * (Jm.transpose()) * Eigen::Vector2d(tau_m1, tau_m2);
}

} // namespace ankle
