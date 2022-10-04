/**
 * @file cassert.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief 3Laws specific version of std library's cassert
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */
#ifndef THREELAWS__CASSERT_HPP_
#define THREELAWS__CASSERT_HPP_

#include <cassert>

#ifdef NDEBUG_3LAWS

#define assert3(expr) (static_cast<void>(0))
#define assert3_msg(expr, what) (static_cast<void>(0))

#ifdef __USE_GNU
#define assert_perror3(errnum) (static_cast<void>(0))
#endif

#else  // NDEBUG_3LAWS

#ifndef _ASSERT_H_DECLS
#define _ASSERT_H_DECLS
__BEGIN_DECLS

/* This prints an "Assertion failed" message and aborts.  */
extern void __assert_fail(const char * __assertion,
  const char * __file,
  unsigned int __line,
  const char * __function) __THROW __attribute__((__noreturn__));

/* Likewise, but prints the error text for ERRNUM.  */
extern void __assert_perror_fail(
  int __errnum, const char * __file, unsigned int __line, const char * __function) __THROW
  __attribute__((__noreturn__));

__END_DECLS
#endif /* Not _ASSERT_H_DECLS */

#define assert3(expr) \
  (static_cast<bool>(expr) ? void(0) : __assert_fail(#expr, __FILE__, __LINE__, ASSERT_FUNCTION3L))

#define assert3_msg(expr, what) \
  (static_cast<bool>(expr)      \
      ? void(0)                 \
      : __assert_fail(what ", but: " #expr, __FILE__, __LINE__, ASSERT_FUNCTION3L))

#ifdef __USE_GNU
#define assert_perror3(errnum)       \
  (!(errnum) ? __ASSERT_VOID_CAST(0) \
             : __assert_perror_fail((errnum), __FILE__, __LINE__, ASSERT_FUNCTION3L))
#endif

#define ASSERT_FUNCTION3L __extension__ __PRETTY_FUNCTION__

#endif  // NDEBUG_3LAWS

#endif  // THREELAWS__CASSERT_HPP_
