/**
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#include "3laws/kernel_generator_node_unicycle.hpp"

#include <Eigen/Core>

#include "3laws/msg_utils.hpp"
#include "external/parameters.hpp"

namespace lll {

constexpr size_t NX = 5;
constexpr size_t NU = 2;

class UnicycleDynamics : public lll::AffineDynamicalModelAbstract
{
public:
  explicit UnicycleDynamics(const UnicycleDynamicsParams & prm)
      : AffineDynamicalModelAbstract(NX, NU),
        m_res{std::make_shared<AffineDynamicalModelEvaluationResult>(NX, NU, true)}, m_prm{prm}
  {}

  UnicycleDynamics(const UnicycleDynamics &)             = default;
  UnicycleDynamics(UnicycleDynamics &&)                  = default;
  UnicycleDynamics & operator=(const UnicycleDynamics &) = default;
  UnicycleDynamics & operator=(UnicycleDynamics &&)      = default;
  ~UnicycleDynamics() override                           = default;

  bool has_gradient() const override { return true; }
  size_t nx() const override { return NX; }
  size_t nu() const override { return NU; }
  std::shared_ptr<AffineDynamicalModelEvaluationResult> evaluate_affine(
    const span<const scalar_t, dynamic_extent> x) const override
  {
    const double & v        = x[2];
    const double & theta    = x[3];
    const double & thetaDot = x[4];

    m_res->f[0] = v * std::cos(theta);
    m_res->f[1] = v * std::sin(theta);
    m_res->f[2] = -m_prm.Cv * v;
    m_res->f[3] = thetaDot;
    m_res->f[4] = -m_prm.Cw * thetaDot;

    Eigen::Map<Eigen::MatrixXd> g(m_res->g.data(), Eigen::Index(NX), Eigen::Index(NU));
    g(2, 0) = m_prm.Cv;
    g(3, 1) = m_prm.Cw;

    Eigen::Map<Eigen::MatrixXd> df_dx(m_res->df_dx.data(), Eigen::Index(NX), Eigen::Index(NX));
    df_dx(0, 2) = std::cos(theta);
    df_dx(0, 3) = -v * std::sin(theta);
    df_dx(1, 2) = std::sin(theta);
    df_dx(1, 3) = v * std::cos(theta);
    df_dx(2, 2) = -m_prm.Cv;
    df_dx(3, 4) = 1.;
    df_dx(4, 4) = -m_prm.Cw;

    return m_res;
  }

private:
  std::shared_ptr<AffineDynamicalModelEvaluationResult> m_res;
  UnicycleDynamicsParams m_prm;
};

class UnicycleMap : public SafetyMapAbstract
{
public:
  explicit UnicycleMap(const UnicycleMapParams & prm)
      : m_res{std::make_shared<SafetyMapProbingResult>(NX, prm.n_obstacles, false)}, m_prm{prm}
  {}
  UnicycleMap(const UnicycleMap &)             = default;
  UnicycleMap(UnicycleMap &&)                  = default;
  UnicycleMap & operator=(const UnicycleMap &) = default;
  UnicycleMap & operator=(UnicycleMap &&)      = default;
  ~UnicycleMap() override                      = default;

  bool has_gradient() const override { return false; }
  size_t nx() const override { return NX; }
  size_t n_obstacles() const override { return m_prm.n_obstacles; }

  std::shared_ptr<SafetyMapProbingResult> probe(
    const span<const scalar_t, dynamic_extent>, const t_t = 0) const override
  {
    return m_res;
  }

  double min_distance() const { return m_minDist; }
  double min_angle() const { return m_minAngle; }

private:
  std::shared_ptr<SafetyMapProbingResult> m_res;
  UnicycleMapParams m_prm;
  double m_minDist  = 10.;
  double m_minAngle = 0.;
};

class UnicycleFailsafe : public FailsafePolicyAbstract
{
public:
  explicit UnicycleFailsafe(const UnicycleFailsafeParams & prm, std::shared_ptr<UnicycleMap> map)
      : m_res{std::make_shared<FailsafePolicyEvaluationResult>(NX, NU, false)},
        m_map{std::move(map)}, m_prm{prm}
  {}

  UnicycleFailsafe(const UnicycleFailsafe &)             = default;
  UnicycleFailsafe(UnicycleFailsafe &&)                  = default;
  UnicycleFailsafe & operator=(const UnicycleFailsafe &) = default;
  UnicycleFailsafe & operator=(UnicycleFailsafe &&)      = default;
  ~UnicycleFailsafe() override                           = default;

  bool has_gradient() const override { return false; }
  size_t nx() const override { return NX; }
  size_t nu() const override { return NU; }

  std::shared_ptr<FailsafePolicyEvaluationResult> evaluate(
    const span<const scalar_t, dynamic_extent>, const t_t = 0) const override
  {
    auto & ub = m_res->val;
    std::fill(ub.begin(), ub.end(), 0.);

    const double minDist  = m_map->min_distance();
    const double minAngle = m_map->min_angle();

    double direction = 1.0;

    if (minDist < 2. * m_prm.R) {  // move away from obstacle
      if (std::fabs(minAngle) < M_PI_2) {
        direction = -1.0;
      } else {
        direction = 1.0;
      }

      ub[0] = direction * m_prm.Kv * std::fabs(minDist - 2. * m_prm.R);

      if (direction < 0.) {
        ub[1] = m_prm.Kw * minAngle;
      } else {
        double tmp = std::fmod(minAngle, std::copysign(1., minAngle) * M_PI);
        if (tmp > 0) {
          tmp -= M_PI;
        } else {
          tmp += M_PI;
        }
        ub[1] = m_prm.Kw * tmp;
      }
    }
    if (ub[0] > m_prm.max_v) { ub[0] = m_prm.max_v; }
    if (ub[0] < -m_prm.max_v) { ub[0] = -m_prm.max_v; }

    return m_res;
  }

private:
  std::shared_ptr<FailsafePolicyEvaluationResult> m_res;
  std::shared_ptr<UnicycleMap> m_map;
  UnicycleFailsafeParams m_prm;
};

KernelGeneratorNodeUnicycle::KernelGeneratorNodeUnicycle(const rclcpp::NodeOptions & options)
    : KernelGeneratorNode(options)
{}

void KernelGeneratorNodeUnicycle::init()
{
  cbr::initParams(*this, "", m_prm);

  auto dyn      = std::make_shared<UnicycleDynamics>(m_prm.dynamics);
  auto map      = std::make_shared<UnicycleMap>(m_prm.map);
  auto failsafe = std::make_shared<UnicycleFailsafe>(m_prm.failsafe, map);

  initialize(m_prm.kernel_generator, dyn, map, failsafe);
}

}  // namespace lll
