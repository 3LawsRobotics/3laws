/**
 * @file memory.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief C++11 implementation of std::make_unique.
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */
#ifndef THREELAWS_MEMORY_HPP
#define THREELAWS_MEMORY_HPP

#include <memory>
#include <type_traits>
#include <utility>

namespace lll {

/// @cond undocumented

namespace detail {

template<typename T>
struct MakeUniq
{
  typedef std::unique_ptr<T> single_object;
};

template<typename T>
struct MakeUniq<T[]>
{
  typedef std::unique_ptr<T[]> array;
};

template<typename T, size_t Bound>
struct MakeUniq<T[Bound]>
{
  struct invalid_type
  {};
};

}  // namespace detail

/// @endcond

/// std::make_unique for single objects
template<typename T, typename... Args>
inline typename detail::MakeUniq<T>::single_object make_unique(Args &&... args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

/// std::make_unique for arrays of unknown bound
template<typename T>
inline typename detail::MakeUniq<T>::array make_unique(std::size_t num)
{
  return std::unique_ptr<T>(new typename std::remove_extent<T>::type[num]());
}

/// Disable std::make_unique for arrays of known bound
template<typename T, typename... Args>
typename detail::MakeUniq<T>::invalid_type make_unique(Args &&...) = delete;

}  // namespace lll

#endif  // THREELAWS_MEMORY_HPP
