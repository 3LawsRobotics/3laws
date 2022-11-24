/**
 * @file safety_map_lidar.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */
#ifndef THREELAWS_SAFETY_MAP_OBSTACLES_HPP
#define THREELAWS_SAFETY_MAP_OBSTACLES_HPP

#include <array>
#include <memory>
#include <string>

#include "3laws/safety_map_abstract.hpp"

namespace lll {

/**
 * @brief SafetyMap for a 3D spherical obsctacles
 * @details Concrete implementation of SafetyMapAbstract for 3 dimensional spherical obstacles.
 *
 * This safety map is appropriate when detection and localization of obstacles from sensor data is
 * available.
 *
 * Obstacles can either be inactive or active, in which case their geometry is a sphere specified by
 * its radius.
 *
 * Obstacles have a position and a velocity speficied at a given time which is
 * extrapolated when probing the map at various times.
 *
 * For this safety map:
 * - has_gradient() returns `true`
 * - nx() returns `3`
 * - n_obstacles() returns `n_obstacles` specified at construction
 */
class SafetyMapObstacles : public SafetyMapAbstract
{
public:
  /**
   * @brief Obstacle data
   */
  struct Obstacle
  {
    std::string name = "obstacle";  ///< Name of obstacle
    bool act         = false;       ///< Obstacle is active
    t_t t            = 0;           ///< Time at which position and velocity have been measured
    std::array<scalar_t, 3> pos{{0., 0., 0.}};  ///< Obstacle position
    std::array<scalar_t, 3> vel{{0., 0., 0.}};  ///< Obstacle velocity
    scalar_t rad = 0.;                          ///< Minimum safe distance to obstacle
  };

  SafetyMapObstacles()                                       = delete;
  SafetyMapObstacles(const SafetyMapObstacles &)             = delete;
  SafetyMapObstacles(SafetyMapObstacles &&)                  = default;
  SafetyMapObstacles & operator=(const SafetyMapObstacles &) = delete;
  SafetyMapObstacles & operator=(SafetyMapObstacles &&)      = default;
  ~SafetyMapObstacles();

  /**
   * @brief Construct a new Safety Map Obstacles object with a specific number of obstacles
   *
   * @param n_obstacles Number of obstacles
   */
  explicit SafetyMapObstacles(const size_t n_obstacles);

  bool has_gradient() const override;
  size_t n_obstacles() const override;
  size_t nx() const override;
  std::shared_ptr<lll::SafetyMapProbingResult> probe(
    const lll::span<const lll::scalar_t, lll::dynamic_extent> x,
    const lll::t_t t_nsec = 0) const override;

  /**
   * @brief Const accessor to obstacles
   *
   * @param idx Index of obstacle
   * @return const Obstacle&
   */
  const Obstacle & obstacle(const size_t idx) const;

  /**
   * @brief Accessor to obstacles
   *
   * @param idx Index of obstacle
   * @return Obstacle&
   */
  Obstacle & obstacle(const size_t idx);

private:
  std::unique_ptr<struct SafetyMapObstaclesImpl> m_pImpl;  ///< Pointer to implementation
};

}  // namespace lll

#endif  // THREELAWS_SAFETY_MAP_OBSTACLES_HPP
