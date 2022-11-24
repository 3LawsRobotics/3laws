/**
 * @file safety_map_lidar.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */
#ifndef THREELAWS_SAFETY_MAP_POINTCLOUD_HPP
#define THREELAWS_SAFETY_MAP_POINTCLOUD_HPP

#include <memory>

#include "3laws/safety_map_abstract.hpp"

namespace lll {

/**
 * @brief SafetyMap for a 3D pointcloud data
 * @details Concrete implementation of SafetyMapAbstract for 3 dimensional pointcloud data. The
 * pointcloud is assumed to be broken up into clusters that get each get updated entirely at once.
 *
 * Clusters that have not yet been updated are considered empty.
 *
 * This safety map is appropriate when using multiple unstructured pointcloud sources like forward
 * facing lidars in a mostly static environment. The pointcloud data is assumed to be static between
 * updates and provided in world coordinates, so consistent localization is required.
 *
 * For this safety map:
 * - has_gradient() returns `true`
 * - nx() returns `3`
 * - n_obstacles() returns `n_clusters * n_pts_per_cluster`
 */
class SafetyMapPointcloud : public SafetyMapAbstract
{
public:
  SafetyMapPointcloud()                                        = delete;
  SafetyMapPointcloud(const SafetyMapPointcloud &)             = delete;
  SafetyMapPointcloud(SafetyMapPointcloud &&)                  = default;
  SafetyMapPointcloud & operator=(const SafetyMapPointcloud &) = delete;
  SafetyMapPointcloud & operator=(SafetyMapPointcloud &&)      = default;
  ~SafetyMapPointcloud();

  /**
   * @brief Construct a new Safety Map Pointcloud object with uniform pointcloud clusters size
   *
   * @param n_clusters Number of pointcloud clusters
   * @param n_pts_per_cluster Number of points per cluster
   * @param pt_size Minimum safe distance to pointcloud points
   */
  SafetyMapPointcloud(
    const size_t n_clusters, const size_t n_pts_per_cluster, const scalar_t pt_size = 0.);

  /**
   * @brief Construct a new Safety Map Pointcloud object with non-uniform pointcloud clusters size
   *
   * @param cluster_sizes List of cluster sizes
   * @param pt_size Minimum safe distance to pointcloud points
   */
  SafetyMapPointcloud(
    const span<const size_t, dynamic_extent> cluster_sizes, const scalar_t pt_size = 0.);

  bool has_gradient() const override;
  size_t n_obstacles() const override;
  size_t nx() const override;
  std::shared_ptr<lll::SafetyMapProbingResult> probe(
    const lll::span<const lll::scalar_t, lll::dynamic_extent> x,
    const lll::t_t t_nsec = 0) const override;

  /**
   * @brief Update pointcloud cluster
   *
   * @param idx Index of the cluster
   * @param pts Points of the cluster (number of points must be equal to size specified at
   * construction)
   */
  void update_cluster(
    const size_t idx, const span<const span<const scalar_t, 3>, dynamic_extent> pts);

  /**
   * @brief Accessor to cluster point
   *
   * @param cluster_idx Index of cluster
   * @param point_idx Index of point in cluster
   * @return std::array<scalar_t, 3>&
   */
  std::array<scalar_t, 3> & cluster_point(const size_t cluster_idx, const size_t point_idx);

  /**
   * @brief Const accessor to cluster point
   *
   * @param cluster_idx Index of cluster
   * @param point_idx Index of point in cluster
   * @return const std::array<scalar_t, 3>&
   */
  const std::array<scalar_t, 3> & cluster_point(
    const size_t cluster_idx, const size_t point_idx) const;

  /**
   * @brief Get size of cluster
   *
   * @param idx Index of the cluster
   */
  size_t get_cluster_size(const size_t idx);

  /**
   * @brief Return wether or not cluster has ever been updated
   *
   * @param idx Index of the cluster
   */
  bool cluster_has_been_updated(const size_t idx);

private:
  std::unique_ptr<struct SafetyMapPointcloudImpl> m_pImpl;  ///< Pointer to implementation
};

}  // namespace lll

#endif  // THREELAWS_SAFETY_MAP_POINTCLOUD_HPP
