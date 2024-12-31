// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "Eigen/Core"
#include "gtsam/base/VectorSpace.h"

namespace ptc
{
struct Scalar : public Eigen::Matrix<double, 1, 1>
{
  enum { dimension = 1 };

  using Matrix<double, 1, 1>::Matrix;

  GTSAM_EXPORT void print(const std::string& = "") const
  {}

  GTSAM_EXPORT [[nodiscard]] bool equals(const Scalar& s, const double tol = 1e-9) const
  {
    return std::abs(*this - s(0)) < tol;
  }

  [[nodiscard]] const Eigen::Matrix<double, 1, 1>& vector() const
  {
    return *this;
  }

  [[nodiscard]] operator double() const // NOLINT
  {
    return this->operator()(0);
  }

  [[nodiscard]] static Scalar identity()
  {
    return Eigen::Matrix<double, 1, 1>::Identity();
  }
};

using PositiveScalar = Scalar;
}

template<>
struct gtsam::traits<ptc::Scalar>
: public gtsam::internal::VectorSpace<ptc::Scalar>
{};
