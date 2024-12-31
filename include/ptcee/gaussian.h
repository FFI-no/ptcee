// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "ptcee/covariance.h"
#include "ptcee/vector_traits.h"

namespace ptc
{
template<
  typename State,
  int dim = vector_traits<State>::dim,
  typename Real = typename vector_traits<State>::Real
>
struct Gaussian
{
  State x;
  Covariance<dim, Real> P;
};

// Specialization for scalars
template<typename Real>
struct Gaussian<Real, 1, Real>
{
  template<typename PExpr>
  Gaussian(
    const Real x_in,
    const PExpr& P_in
  )
    : x{x_in}
    , P{P_in}
  {}

  Real x;
  Covariance<1, Real> P;
};

template<typename State>
[[nodiscard]] bool operator==(
  const Gaussian<State>& lhs,
  const Gaussian<State>& rhs
);

// ----- Implementation -----
template<typename State>
bool operator==(
  const Gaussian<State>& lhs,
  const Gaussian<State>& rhs
)
{
  return lhs.x == rhs.x && lhs.P == rhs.P;
}
}

