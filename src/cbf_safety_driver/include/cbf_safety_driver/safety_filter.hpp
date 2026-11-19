#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

namespace cbf_safety_driver {

enum class SafetyMode {
  None,
  Speed250,
  CbfAvoid,
  CbfForce,
};

struct IsoRegion {
  double f_max;       // Предел силы по ISO/TS 15066 [Н]
  double stiffness;   // Жёсткость ткани k [Н/м]
  double eta;         // Запас на вклад привода
};

struct SafetyParams {
  double v_limit = 0.25;          // Лимит скорости Speed250 [м/с]
  double alpha = 4.0;             // Коэффициент CBF класса K [1/с]
  double d_active = 0.25;         // Радиус активации барьера [м]
  double d_safe = 0.02;           // Запас геометрического избегания [м]
  double r_reach = 0.01;          // Запас на задержку восприятия [м]
  double eff_mass = 3.5;          // Приведённая масса пары μ [кг]
  double qdot_max = 2.0;          // Предел скорости суставов [рад/с]
  double slack_tol = 1e-6;        // Порог slack для аварийной остановки
};

struct FilterInput {
  Eigen::Vector3d v_desired = Eigen::Vector3d::Zero();
  Eigen::Vector3d tcp_pos = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 3, 7> jac_tcp = Eigen::Matrix<double, 3, 7>::Zero();
  bool zone_visible = false;
  std::string zone_name = "head";
  Eigen::Vector3d zone_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d zone_vel = Eigen::Vector3d::Zero();
  double zone_radius = 0.095;
};

struct FilterOutput {
  Eigen::Matrix<double, 7, 1> qdot_ref = Eigen::Matrix<double, 7, 1>::Zero();
  bool intervened = false;
  bool emergency_stop = false;
  double min_barrier_residual = 0.0;
  double slack = 0.0;
  double qp_ms = 0.0;
};

IsoRegion region_for_zone(const std::string& zone_name);
double compute_vmax(const IsoRegion& region, double eff_mass);
SafetyMode parse_safety_mode(const std::string& mode);
std::string to_string(SafetyMode mode);

class SafetyFilter {
public:
  virtual ~SafetyFilter() = default;
  virtual SafetyMode mode() const = 0;
  virtual FilterOutput apply(const FilterInput& in) const = 0;
};

class NoFilter final : public SafetyFilter {
public:
  explicit NoFilter(SafetyParams params);
  SafetyMode mode() const override { return SafetyMode::None; }
  FilterOutput apply(const FilterInput& in) const override;
private:
  SafetyParams params_;
};

class SpeedLimitFilter final : public SafetyFilter {
public:
  explicit SpeedLimitFilter(SafetyParams params);
  SafetyMode mode() const override { return SafetyMode::Speed250; }
  FilterOutput apply(const FilterInput& in) const override;
private:
  SafetyParams params_;
};

class ApproachCbfFilter : public SafetyFilter {
public:
  explicit ApproachCbfFilter(SafetyParams params);
  FilterOutput apply(const FilterInput& in) const override;
protected:
  virtual double approach_cap(const FilterInput& in, double dist) const = 0;
  SafetyParams params_;
};

class CbfAvoidFilter final : public ApproachCbfFilter {
public:
  using ApproachCbfFilter::ApproachCbfFilter;
  SafetyMode mode() const override { return SafetyMode::CbfAvoid; }
protected:
  double approach_cap(const FilterInput& in, double dist) const override;
};

class CbfForceFilter final : public ApproachCbfFilter {
public:
  using ApproachCbfFilter::ApproachCbfFilter;
  SafetyMode mode() const override { return SafetyMode::CbfForce; }
protected:
  double approach_cap(const FilterInput& in, double dist) const override;
};

std::unique_ptr<SafetyFilter> make_safety_filter(
    SafetyMode mode, const SafetyParams& params = SafetyParams{});

}  // namespace cbf_safety_driver
