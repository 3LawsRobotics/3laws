/**
 * @file safety_map_lidar.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */
#ifndef THREELAWS_SAFETY_MAP_COLLECTION_HPP
#define THREELAWS_SAFETY_MAP_COLLECTION_HPP

#include <memory>
#include <utility>
#include <vector>

#include "3laws/safety_map_abstract.hpp"

namespace lll {

/**
 * @brief SafetyMap for merging multiple safety maps together
 * @details Concrete implementation of SafetyMapAbstract for a collection of safety maps.
 *
 * This safety map is appropriate when multiple sensing modalities are used in conjunction.
 *
 * For this safety map:
 * - has_gradient() returns `true` if all safety maps in the collection have a gradient, otherwise
 * returns `false`
 * - nx() returns `nx` specified at construction
 * - n_obstacles() returns the sum of n_obstacles() of all the safety maps in the collection
 */
class SafetyMapCollection : public SafetyMapAbstract
{
public:
  SafetyMapCollection()                                        = delete;
  SafetyMapCollection(const SafetyMapCollection &)             = delete;
  SafetyMapCollection(SafetyMapCollection &&)                  = default;
  SafetyMapCollection & operator=(const SafetyMapCollection &) = delete;
  SafetyMapCollection & operator=(SafetyMapCollection &&)      = default;
  ~SafetyMapCollection();

  /**
   * @brief Construct a new Safety Map Collection object
   * @details Each safety map is associated with an index vector, which is the vector of indexes of
   * the system state to be used to probe this particular map. Therefore, the length of this index
   * vector must be equal to the value returned by map.nx(), and it cannot contain values greater
   * than the number of system states.
   *
   * @param nx Number of system states
   * @param maps List of pairs of maps and associated index vector
   */
  SafetyMapCollection(const size_t nx,
    std::initializer_list<std::pair<SafetyMapAbstract::SharedPtr, std::vector<size_t>>> maps);

  bool has_gradient() const override;
  size_t n_obstacles() const override;
  size_t nx() const override;
  std::shared_ptr<SafetyMapProbingResult> probe(
    const span<const scalar_t, dynamic_extent> x, const t_t t_nsec = 0) const override;

private:
  std::unique_ptr<struct SafetyMapCollectionImpl> m_pImpl;  ///< Pointer to implementation
};

}  // namespace lll

#endif  // THREELAWS_SAFETY_MAP_COLLECTION_HPP
