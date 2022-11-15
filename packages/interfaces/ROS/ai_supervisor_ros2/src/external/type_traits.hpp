// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_utils/blob/master/LICENSE

/** @file */

#ifndef CBR_UTILS__TYPE_TRAITS_HPP_
#define CBR_UTILS__TYPE_TRAITS_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace cbr {

/***************************************************************************
 * \brief Returns type of variable as string_view
 ***************************************************************************/

/**
 * @brief Get string representing of a type.
 *
 * @tparam T Type to get the string representation for.
 * @return Returns an std::string_view representing type T.
 */
template<typename T>
constexpr std::string_view type_name() noexcept
{
  std::string_view name = "Error: unsupported compiler";
  std::string_view prefix;
  std::string_view suffix;

#ifdef __clang__
  name   = __PRETTY_FUNCTION__;
  prefix = "auto cbr::type_name() [T = ";
  suffix = "]";
#elif defined(__GNUC__)
  name   = __PRETTY_FUNCTION__;
  prefix = "constexpr auto cbr::type_name() [with T = ";
  suffix = "]";
#elif defined(_MSC_VER)
  name   = __FUNCSIG__;
  prefix = "auto __cdecl cbr::type_name<";
  suffix = ">(void) noexcept";
#endif

  name.remove_prefix(prefix.size());
  name.remove_suffix(suffix.size());

  return name;
}

/***************************************************************************
 * \brief Wrapper for static assertion over parameter pack
 ***************************************************************************/

/**
 * @brief Check if typetrait predicate is valid/true for all types.
 * @details See std::conjunction for what constitutes a valid predicate.
 *
 * @tparam T Predicate.
 * @tparam Ts Types to apply predicate on.
 * @return Boolean value, true if all T<Ts>... are valid.
 */
template<template<typename> typename T, typename... Ts>
constexpr bool static_assert_all()
{
  return std::conjunction_v<T<Ts>...>;
}

/// @cond

template<typename T, bool B = std::is_enum<T>::value>
struct is_scoped_enum : std::false_type
{};

template<typename T>
struct is_scoped_enum<T, true>
    : std::integral_constant<bool,
        !std::is_convertible<T, typename std::underlying_type<T>::type>::value>
{};

/// @endcond

/**
 * @brief Check if type is a scoped enum.
 */
template<class T>
inline constexpr bool is_scoped_enum_v = is_scoped_enum<T>::value;

// Specialization class type trait
/// @cond
template<typename Test, template<typename...> class Ref>
struct is_specialization : std::false_type
{};

template<template<typename...> class Ref, typename... Args>
struct is_specialization<Ref<Args...>, Ref> : std::true_type
{};

/// @endcond

/**
 * @brief Check if type is a specialisation of another type.
 * @details For example:
 * ```
 * constexpr auto v1 = is_specialization_v<std::vector<double>,std::vector>;
 * constexpr auto v2 = is_specialization_v<std::vector<double>,std::tuple>;
 * ```
 * Then v1 == true but v2 == false.
 *
 * @tparam T Type to check if specialization of Ref or not.
 * @tparam Ref Reference type to check if T is a specialization of or not.
 */
template<typename T, template<typename...> class Ref>
inline constexpr bool is_specialization_v = is_specialization<T, Ref>::value;

// std::vector
/// @cond
template<typename>
struct is_std_vector : std::false_type
{};

template<typename T, typename... Ts>
struct is_std_vector<std::vector<T, Ts...>> : std::true_type
{};
/// @endcond

/**
 * @brief Check if type is an std::vector.
 */
template<typename T>
inline constexpr bool is_std_vector_v = is_std_vector<T>::value;

// std::array
/// @cond
template<typename, bool = false>
struct is_std_array : std::false_type
{};

template<typename T, std::size_t S>
struct is_std_array<std::array<T, S>> : std::true_type
{};
/// @endcond

/**
 * @brief Check if type is an std::vector.
 */
template<typename T>
inline constexpr bool is_std_array_v = is_std_array<T>::value;

// std::pair
/// @cond
template<typename>
struct is_std_pair : std::false_type
{};

template<typename T1, typename T2>
struct is_std_pair<std::pair<T1, T2>> : std::true_type
{};
/// @endcond

/**
 * @brief Check if type is an std::vector.
 */
template<typename T>
inline constexpr bool is_std_pair_v = is_std_pair<T>::value;

// std::tuple
/// @cond
template<typename>
struct is_std_tuple : std::false_type
{};

template<typename... Ts>
struct is_std_tuple<std::tuple<Ts...>> : std::true_type
{};
/// @endcond

/**
 * @brief Check if type is an std::vector.
 */
template<typename T>
inline constexpr bool is_std_tuple_v = is_std_tuple<T>::value;

// std::variant
/// @cond
template<typename>
struct is_std_variant : std::false_type
{};

template<typename... Ts>
struct is_std_variant<std::variant<Ts...>> : std::true_type
{};
/// @endcond

/**
 * @brief Check if type is an std::vector.
 */
template<typename T>
inline constexpr bool is_std_variant_v = is_std_variant<T>::value;

// std::shared_ptr
/// @cond
template<typename>
struct is_std_shared_ptr : std::false_type
{};

template<typename T>
struct is_std_shared_ptr<std::shared_ptr<T>> : std::true_type
{};
/// @endcond

/**
 * @brief Check if type is an std::vector.
 */
template<typename T>
inline constexpr bool is_std_shared_ptr_v = is_std_shared_ptr<T>::value;

// Chrono duration
/// @cond
template<typename T>
struct is_chrono_duration : std::false_type
{};

template<typename R, typename P>
struct is_chrono_duration<std::chrono::duration<R, P>> : std::true_type
{};
/// @endcond

/**
 * @brief Check if type is an std::vector.
 */
template<typename T>
inline constexpr bool is_chrono_duration_v = is_chrono_duration<T>::value;

// std::unique_ptr
/// @cond
template<typename>
struct is_std_unique_ptr : std::false_type
{};

template<typename T>
struct is_std_unique_ptr<std::unique_ptr<T>> : std::true_type
{};
/// @endcond

/**
 * @brief Check if type is an std::vector.
 */
template<typename T>
inline constexpr bool is_std_unique_ptr_v = is_std_unique_ptr<T>::value;

// std::optional
/// @cond
template<typename>
struct is_std_optional : std::false_type
{};

template<typename T>
struct is_std_optional<std::optional<T>> : std::true_type
{};
/// @endcond

/**
 * @brief Check if type is an std::vector.
 */
template<typename T>
inline constexpr bool is_std_optional_v = is_std_optional<T>::value;

/**
 * @brief Always false constexpr constant.
 * @details Usefull to allow compilation of:
 * ```
 * template<class T>
 * void foo()
 * {
 *    if constexpr (std::is_signed_v<T>) {
 *      // doStuff...
 *    } else {
 *      static_assert(false_v<T>,"Unsupported type");
 *    }
 * }
 * ```
 * See https://quuxplusone.github.io/blog/2018/04/02/false-v/ for more info.
 */
template<typename...>
inline constexpr bool false_v = false;

/**
 * @brief Always true constexpr constant.
 * @details Usefull to allow compilation of:
 * ```
 * template<class T>
 * void foo()
 * {
 *    if constexpr (std::is_signed_v<T>) {
 *      // doStuff...
 *    } else {
 *      static_assert(!true_v<T>,"Unsupported type");
 *    }
 * }
 * ```
 * See https://quuxplusone.github.io/blog/2018/04/02/false-v/ for more info.
 */
template<typename...>
inline constexpr bool true_v = true;

}  // namespace cbr

#endif  // CBR_UTILS__TYPE_TRAITS_HPP_
