#include <gtest/gtest.h>

#include <stdexcept>

#include "cbf_safety_driver/safety_filter.hpp"

namespace {

using cbf_safety_driver::FilterInput;
using cbf_safety_driver::SafetyMode;
using cbf_safety_driver::SafetyParams;
using cbf_safety_driver::compute_vmax;
using cbf_safety_driver::make_safety_filter;
using cbf_safety_driver::region_for_zone;

FilterInput make_head_approach_input(double v_des_x, double zone_x, double zone_vx = 0.0) {
  FilterInput in;
  in.v_desired = Eigen::Vector3d(v_des_x, 0.0, 0.0);
  in.tcp_pos = Eigen::Vector3d::Zero();
  in.jac_tcp.setZero();
  in.jac_tcp(0, 0) = 1.0;
  in.jac_tcp(1, 1) = 1.0;
  in.jac_tcp(2, 2) = 1.0;
  in.zone_visible = true;
  in.zone_name = "head";
  in.zone_center = Eigen::Vector3d(zone_x, 0.0, 0.0);
  in.zone_vel = Eigen::Vector3d(zone_vx, 0.0, 0.0);
  in.zone_radius = 0.095;
  return in;
}

TEST(SafetyFilterTest, IsoVmaxMatchesProtocol) {
  const auto head = region_for_zone("head");
  const double v_max = compute_vmax(head, 3.5);
  EXPECT_NEAR(v_max, 0.1269, 1e-3);
}

TEST(SafetyFilterTest, SpeedLimitCapsAt250MmPerSec) {
  auto layer = make_safety_filter(SafetyMode::Speed250);
  FilterInput in = make_head_approach_input(0.5, 1.0);
  in.zone_visible = false;
  const auto out = layer->apply(in);
  EXPECT_TRUE(out.intervened);
  EXPECT_LE(out.qdot_ref[0], 0.25);
  EXPECT_NEAR(out.qdot_ref[0], 0.25, 1e-4);
}

TEST(SafetyFilterTest, CbfForceLimitsApproachAndPreservesTangent) {
  SafetyParams params;
  params.eff_mass = 3.5;
  params.r_reach = 0.01;
  auto layer = make_safety_filter(SafetyMode::CbfForce, params);

  FilterInput in = make_head_approach_input(0.5, 0.105);  // Зазор до поверхности 0.01 м == r_reach
  in.v_desired.y() = 0.2;
  const auto out = layer->apply(in);

  const auto head = region_for_zone("head");
  const double expected_cap = head.eta * compute_vmax(head, params.eff_mass);
  EXPECT_TRUE(out.intervened);
  EXPECT_FALSE(out.emergency_stop);
  EXPECT_LE(out.qdot_ref[0], expected_cap + 1e-4);
  EXPECT_NEAR(out.qdot_ref[1], 0.2, 1e-4);
}

TEST(SafetyFilterTest, EmergencyStopTriggeredWhenBarrierInfeasible) {
  SafetyParams params;
  params.qdot_max = 0.2;
  auto layer = make_safety_filter(SafetyMode::CbfAvoid, params);

  // Зона сближается быстрее предела отхода суставов (-0.5 м/с против 0.2 рад/с).
  FilterInput in = make_head_approach_input(0.1, 0.10, -0.5);
  const auto out = layer->apply(in);
  EXPECT_TRUE(out.intervened);
  EXPECT_TRUE(out.emergency_stop);
  EXPECT_GT(out.slack, 0.0);
}

TEST(SafetyFilterTest, FactoryRejectsUnknownMode) {
  EXPECT_THROW(cbf_safety_driver::parse_safety_mode("invalid_layer"), std::invalid_argument);
}

}  // namespace
