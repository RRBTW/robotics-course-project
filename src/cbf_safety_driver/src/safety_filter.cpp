#include "cbf_safety_driver/safety_filter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace cbf_safety_driver {
namespace {

using Vec7 = Eigen::Matrix<double, 7, 1>;

Vec7 clamp_qdot(const Vec7& qdot, double limit) {
  return qdot.cwiseMax(-limit).cwiseMin(limit);
}

Vec7 solve_damped_ik(const Eigen::Matrix<double, 3, 7>& jac, const Eigen::Vector3d& v_des, double qdot_max) {
  // Демпфированная обратная кинематика для 7-степенного манипулятора.
  const Eigen::Matrix3d jjt = jac * jac.transpose() + 1e-4 * Eigen::Matrix3d::Identity();
  const Vec7 qdot = jac.transpose() * jjt.ldlt().solve(v_des);
  return clamp_qdot(qdot, qdot_max);
}

// Точная QP-проекция на полупространство и ограничения скоростей суставов.
bool project_halfspace_box(const Vec7& q_nom, const Vec7& a, double rhs, double q_max,
                           Vec7& q_out, double& slack_out) {
  const Vec7 q0 = clamp_qdot(q_nom, q_max);
  if (a.dot(q0) <= rhs) {
    q_out = q0;
    slack_out = 0.0;
    return false;
  }
  const double a_l1 = a.cwiseAbs().sum();
  const double min_achievable = -q_max * a_l1;
  if (min_achievable > rhs) {
    for (int i = 0; i < 7; ++i) {
      q_out[i] = (a[i] > 0.0) ? -q_max : (a[i] < 0.0 ? q_max : q0[i]);
    }
    slack_out = min_achievable - rhs;
    return true;
  }
  auto eval_q = [&](double lam) {
    return clamp_qdot(q_nom - lam * a, q_max);
  };
  double lo = 0.0;
  double hi = 1.0;
  while (a.dot(eval_q(hi)) > rhs && hi < 1e9) {
    hi *= 2.0;
  }
  for (int it = 0; it < 48; ++it) {
    const double mid = 0.5 * (lo + hi);
    if (a.dot(eval_q(mid)) > rhs) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  q_out = eval_q(hi);
  slack_out = std::max(0.0, a.dot(q_out) - rhs);
  return true;
}

}  // namespace

IsoRegion region_for_zone(const std::string& zone_name) {
  if (zone_name == "head") {
    return IsoRegion{65.0, 75000.0, 0.85};
  }
  if (zone_name == "chest") {
    return IsoRegion{280.0, 25000.0, 0.67};
  }
  if (zone_name == "hand") {
    return IsoRegion{280.0, 75000.0, 0.91};
  }
  throw std::invalid_argument("Unknown ISO/TS 15066 zone: " + zone_name);
}

double compute_vmax(const IsoRegion& region, double eff_mass) {
  if (eff_mass <= 0.0 || region.stiffness <= 0.0) {
    throw std::invalid_argument("Effective mass and stiffness must be positive");
  }
  return region.f_max / std::sqrt(eff_mass * region.stiffness);
}

SafetyMode parse_safety_mode(const std::string& mode) {
  if (mode == "none") return SafetyMode::None;
  if (mode == "speed250") return SafetyMode::Speed250;
  if (mode == "cbf_avoid") return SafetyMode::CbfAvoid;
  if (mode == "cbf_force") return SafetyMode::CbfForce;
  throw std::invalid_argument("Unknown safety mode: " + mode);
}

std::string to_string(SafetyMode mode) {
  switch (mode) {
    case SafetyMode::None: return "none";
    case SafetyMode::Speed250: return "speed250";
    case SafetyMode::CbfAvoid: return "cbf_avoid";
    case SafetyMode::CbfForce: return "cbf_force";
  }
  return "unknown";
}

NoFilter::NoFilter(SafetyParams params) : params_(params) {}

FilterOutput NoFilter::apply(const FilterInput& in) const {
  const auto t0 = std::chrono::steady_clock::now();
  FilterOutput out;
  out.qdot_ref = solve_damped_ik(in.jac_tcp, in.v_desired, params_.qdot_max);
  out.qp_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return out;
}

SpeedLimitFilter::SpeedLimitFilter(SafetyParams params) : params_(params) {}

FilterOutput SpeedLimitFilter::apply(const FilterInput& in) const {
  const auto t0 = std::chrono::steady_clock::now();
  FilterOutput out;
  Eigen::Vector3d v = in.v_desired;
  const double speed = v.norm();
  if (speed > params_.v_limit && speed > 1e-9) {
    v *= params_.v_limit / speed;
    out.intervened = true;
  }
  out.qdot_ref = solve_damped_ik(in.jac_tcp, v, params_.qdot_max);
  out.qp_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return out;
}

ApproachCbfFilter::ApproachCbfFilter(SafetyParams params) : params_(params) {}

FilterOutput ApproachCbfFilter::apply(const FilterInput& in) const {
  const auto t0 = std::chrono::steady_clock::now();
  FilterOutput out;
  const Vec7 q_nom = solve_damped_ik(in.jac_tcp, in.v_desired, params_.qdot_max);
  out.qdot_ref = q_nom;

  if (in.zone_visible) {
    const Eigen::Vector3d delta = in.zone_center - in.tcp_pos;
    const double center_dist = delta.norm();
    const double dist = center_dist - in.zone_radius;
    if (dist <= params_.d_active) {
      const Eigen::Vector3d normal =
          (center_dist > 1e-9) ? Eigen::Vector3d(delta / center_dist) : Eigen::Vector3d(Eigen::Vector3d::UnitX());
      const Vec7 a = in.jac_tcp.transpose() * normal;
      const double cap = approach_cap(in, dist);
      const double rhs = normal.dot(in.zone_vel) + cap;

      double slack = 0.0;
      const bool active = project_halfspace_box(q_nom, a, rhs, params_.qdot_max, out.qdot_ref, slack);
      out.slack = slack;
      out.emergency_stop = (slack > params_.slack_tol);
      if (out.emergency_stop) {
        // Сбрасываем номинальную цель и максимально тормозим от барьера.
        double brake_slack = 0.0;
        project_halfspace_box(Vec7::Zero(), a, rhs, params_.qdot_max, out.qdot_ref, brake_slack);
        out.slack = brake_slack;
      }
      out.min_barrier_residual = rhs - a.dot(out.qdot_ref);
      out.intervened = active || out.emergency_stop;
    }
  }
  out.qp_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return out;
}

double CbfAvoidFilter::approach_cap(const FilterInput& /*in*/, double dist) const {
  return params_.alpha * (dist - params_.d_safe);
}

double CbfForceFilter::approach_cap(const FilterInput& in, double dist) const {
  const IsoRegion region = region_for_zone(in.zone_name);
  const double v_max = compute_vmax(region, params_.eff_mass);
  return region.eta * v_max + params_.alpha * std::max(dist - params_.r_reach, 0.0);
}

std::unique_ptr<SafetyFilter> make_safety_filter(SafetyMode mode, const SafetyParams& params) {
  switch (mode) {
    case SafetyMode::None: return std::make_unique<NoFilter>(params);
    case SafetyMode::Speed250: return std::make_unique<SpeedLimitFilter>(params);
    case SafetyMode::CbfAvoid: return std::make_unique<CbfAvoidFilter>(params);
    case SafetyMode::CbfForce: return std::make_unique<CbfForceFilter>(params);
  }
  throw std::invalid_argument("Unsupported safety mode");
}

}  // namespace cbf_safety_driver
