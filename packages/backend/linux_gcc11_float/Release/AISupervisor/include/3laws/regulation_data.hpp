/**
 * @file regulation_data.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief RegulationData and InputConstraints class definitions
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#ifndef THREELAWS_REGULATION_DATA_HPP
#define THREELAWS_REGULATION_DATA_HPP

#include <algorithm>
#include <memory>
#include <vector>

#include "3laws/common.hpp"

namespace lll {

/**
 * @brief Input constraints
 * @details Polytopic input constraints set define as \f$ \{u \in \mathbb{R}^{\text{nu}} \mid lb
 * \leq M.u \leq ub \} \f$
 *
 */
struct InputConstraints
{
  size_t nu = 0;  ///< Number of inputs
                  /**< Must be strictly positive for object to be valid. */

  size_t n_cstr = 0;  ///< Number of input constraints
                      /**< Can be 0. */

  std::vector<scalar_t> lb;  ///< Lower bounds
                             /**< In column major order, size: n_cstr. */

  std::vector<scalar_t> M;  ///< Transfrom matrix
                            /**< In column major order, size: n_cstr*nu. */

  std::vector<scalar_t> ub;  ///< Upper bounds
                             /**< In column major order, size: n_cstr. */

  InputConstraints()                                     = default;
  InputConstraints(const InputConstraints &)             = default;
  InputConstraints(InputConstraints &&)                  = default;
  InputConstraints & operator=(const InputConstraints &) = default;
  InputConstraints & operator=(InputConstraints &&)      = default;
  ~InputConstraints()                                    = default;

  InputConstraints(const size_t nu_, const size_t n_cstr_)
      : nu{nu_}, n_cstr{n_cstr_}, lb(n_cstr, scalar_t(-1.)), M(n_cstr * nu, scalar_t(0.)),
        ub(n_cstr, scalar_t(1.))
  {
    for (size_t i = 0; i < std::min(nu, n_cstr); ++i) { M[(n_cstr + 1) * i] = scalar_t(1.); }
  }

  /**
   * @brief Initialize input constraint data
   * @details Set nu and n_cstr to specified input values, resize lb, M, and ub accordingly, and set
   * lb values to -1, ub values to 1, M main diagonal values to 1 and its other elements to 0.
   * @param nu Number of inputs (must be strictly positive)
   * @param nCstr Number of constraints
   */
  void init(const size_t nu, const size_t nCstr);

  /**
   * @brief Initialize input constraint data
   * @details Same as init(nu,nu).
   *
   * @param nu Number of inputs
   */
  void init(const size_t nu);

  /**
   * @brief Assert that constraint set data is consitent.
   * @details In particular, check that sizes match and that \f$ lb \leq ub \f$.
   */
  void assert_consistent() const;
};

inline void InputConstraints::init(const size_t nInputs, const size_t nCstr)
{
  assert3_msg(nInputs > 0, "Number of inputs must be strictly positive");

  nu     = nInputs;
  n_cstr = nCstr;
  lb.assign(n_cstr, scalar_t(-1.));
  ub.assign(n_cstr, scalar_t(1.));
  M.assign(n_cstr * nu, scalar_t(0.));
  for (size_t i = 0; i < std::min(nu, nCstr); ++i) { M[(n_cstr + 1) * i] = scalar_t(1.); }
}

inline void InputConstraints::init(const size_t nInputs) { init(nInputs, nInputs); }

inline void InputConstraints::assert_consistent() const
{
  assert3_msg(lb.size() == n_cstr, "lb must be of size n_cstr");
  assert3_msg(M.size() == n_cstr * nu, "M must be of size n_cstr * nu");
  assert3_msg(ub.size() == n_cstr, "ub must be of size n_cstr");
  for (__attribute__((unused)) size_t i = 0; i < n_cstr; ++i) {
    assert3_msg(lb[i] <= ub[i], "lb[i] must be less or equal to ub[i] for all i<n_cstr");
  }
}

/**
 * @brief Input regulation data
 * @details Input level safety constraints data computed by a kernel generator.
 */
struct RegulationData
{
  size_t nu = 0;            ///< Number of inputs
                            /**< Must be strictly positive for regulation data to be valid. */
  size_t n_safetyCstr = 0;  ///< Number of safety constraints
  size_t n_failsafes  = 0;  ///< Number of failsafe policies

  InputConstraints input_cstr;  ///< Input constraints

  std::vector<scalar_t> lfh;  ///< Lie derivatives of map obstacles w.r.t natural dynamics
                              /**< In column major order, size n_safetyCstr,
                                   empty if not available. */

  std::vector<scalar_t> lgh;  ///< Lie derivatives of map obstacles w.r.t actuation dynamics
                              /**< In column major order, size n_safetyCstr*nu,
                                   empty if not available. */

  std::vector<scalar_t> safety_val;  ///< Current values of safety
                                     /**< In column major order,
                                          size n_safetyCstr*max(n_failsafes,1). */

  std::vector<scalar_t> failsafe_input;  ///< Failsafe input values,
                                         /**< In column major order, size nu*n_failsafes,
                                              empty if not available. */

  /**
   * @brief Assert that data is consitent.
   * @details In particular, check that sizes match and that input_cstr is consistent too.
   */
  void assert_consistent() const;

  /**
   * @brief Assert that 2 RegulationData objects have same size
   * @details Only check that all data members are either equal or have same size, but does not
   * check consitency of the overall data.
   *
   * @param r Regulation data to compare against
   */
  void assert_same_size_as(const RegulationData & r) const;

  using SharedPtr = std::shared_ptr<RegulationData>;
};

inline void RegulationData::assert_consistent() const
{
  assert3_msg(nu != input_cstr.nu, "Sizes must be consistent");
  assert3_msg(!lfh.empty() && lfh.size() != n_safetyCstr, "Sizes must be consistent");
  assert3_msg(!lgh.empty() && lgh.size() != n_safetyCstr * nu, "Sizes must be consistent");
  assert3_msg(safety_val.size() != n_safetyCstr * std::max(size_t(1), n_failsafes),
    "Sizes must be consistent");
  assert3_msg(!failsafe_input.empty() && failsafe_input.size() != nu * n_failsafes,
    "Sizes must be consistent");
  input_cstr.assert_consistent();
}

inline void RegulationData::assert_same_size_as(
  const RegulationData & r __attribute__((unused))) const
{
  assert3_msg(nu == r.nu, "Sizes must be consistent");
  assert3_msg(n_safetyCstr == r.n_safetyCstr, "Sizes must be consistent");
  assert3_msg(n_failsafes == r.n_failsafes, "Sizes must be consistent");
  assert3_msg(input_cstr.nu == r.input_cstr.nu, "Sizes must be consistent");
  assert3_msg(input_cstr.n_cstr == r.input_cstr.n_cstr, "Sizes must be consistent");
  assert3_msg(input_cstr.lb.size() == r.input_cstr.lb.size(), "Sizes must be consistent");
  assert3_msg(input_cstr.M.size() == r.input_cstr.M.size(), "Sizes must be consistent");
  assert3_msg(input_cstr.ub.size() == r.input_cstr.ub.size(), "Sizes must be consistent");
  assert3_msg(lfh.size() == r.lfh.size(), "Sizes must be consistent");
  assert3_msg(lgh.size() == r.lgh.size(), "Sizes must be consistent");
  assert3_msg(safety_val.size() == r.safety_val.size(), "Sizes must be consistent");
  assert3_msg(failsafe_input.size() == r.failsafe_input.size(), "Sizes must be consistent");
}

}  // namespace lll

#endif  // THREELAWS_REGULATION_DATA_HPP
