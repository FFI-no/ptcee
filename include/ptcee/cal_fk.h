// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "ptcee/vector_traits.h"
#include "gtsam/base/VectorSpace.h"

namespace ptc
{
struct CalFK
{
  struct FailedToConvergeError : public std::runtime_error
  {
    FailedToConvergeError()
      : std::runtime_error{"CalFK failed to converge"}
    {}
  };

  using Vector = gtsam::Vector;

  double f;
  double k;
  double w;
  double h;

  CalFK(const double f_in, const double k_in, const double w_in, const double h_in)
    : f{f_in}
    , k{k_in}
    , w{w_in}
    , h{h_in}
  {}

  enum {dimension = 2};

  [[nodiscard]] CalFK retract(const Vector& v) const
  {
    return CalFK{f + v(0), k + v(1), w, h};
  }

  [[nodiscard]] Vector localCoordinates(const CalFK& cal) const
  {
    return Eigen::Matrix<double, 2, 1>{cal.f - f, cal.k - k};
  }

  [[nodiscard]] Eigen::Vector2d calibrate(
    const Eigen::Vector2d& uv,
    gtsam::OptionalJacobian<2, 2> J_xy_wrt_fk = boost::none,
    gtsam::OptionalJacobian<2, 2> J_xy_wrt_uv = boost::none
  ) const
  {
    const Eigen::Vector2d xy_ = (uv - Eigen::Vector2d{w/2, h/2})/f;
    Eigen::Vector2d xy = xy_;

    constexpr size_t max_iter = 14;
    for (size_t i = 0; i < max_iter; ++i)
    {
      xy = (xy_/(1 + k*xy.squaredNorm())).eval();

      constexpr auto eps = 1e-10;

      const double err = (uv - uncalibrate(xy)).squaredNorm();

      if (err < eps)
      {
        break;
      }

      if (i + 1 == max_iter)
      {
        throw FailedToConvergeError{};
      }
    }

    if (J_xy_wrt_fk || J_xy_wrt_uv)
    {
      Eigen::Matrix2d J_uv_wrt_fk;
      Eigen::Matrix2d J_uv_wrt_xy;
      uncalibrate(xy, J_uv_wrt_fk, J_uv_wrt_xy);

      const auto inv_J_uv_wrt_xy = J_uv_wrt_xy.inverse().eval();

      if (J_xy_wrt_fk)
      {
        *J_xy_wrt_fk = -inv_J_uv_wrt_xy*J_uv_wrt_fk;
      }

      if (J_xy_wrt_uv)
      {
        *J_xy_wrt_uv = inv_J_uv_wrt_xy;
      }
    }

    return xy;
  }

  /// \brief Convert to normalized coordinates xy to pixel coordinates uv
  Eigen::Vector2d uncalibrate(
    const Eigen::Vector2d& xy,
    gtsam::OptionalJacobian<2, 2> J_uv_wrt_fk = boost::none,
    gtsam::OptionalJacobian<2, 2> J_uv_wrt_xy = boost::none
  ) const
  {
    const double rr = xy.squaredNorm();
    const double g = 1. + k*rr;

    if (J_uv_wrt_fk)
    {
      J_uv_wrt_fk->col(0) = g*xy;
      J_uv_wrt_fk->col(1) = xy*f*rr;
    }

    if (J_uv_wrt_xy)
    {
      const auto x = xy.x();
      const auto y = xy.y();

      J_uv_wrt_xy->col(0) = Eigen::Vector2d{
        2*f*k*x*x + f*g,
        2*f*k*x*y
      };
      J_uv_wrt_xy->col(1) = Eigen::Vector2d{
        2*f*k*x*y,
        2*f*k*y*y + f*g
      };
    }

    return {
      xy.x()*f*g + w/2,
      xy.y()*f*g + h/2
    };
  }

  [[nodiscard]] Eigen::Matrix<double, 2, 1> vector() const
  {
    return Eigen::Matrix<double, 2, 1>{f, k};
  }

  void print(const std::string&) const {}

  [[nodiscard]] bool equals(const CalFK& other, const double tol) const
  {
    return std::abs(f - other.f) < tol && std::abs(k - other.k) < tol;
  }
};
}

template<>
struct gtsam::traits<ptc::CalFK> : public gtsam::internal::Manifold<ptc::CalFK>
{};

template<>
struct ptc::vector_traits<ptc::CalFK>
{
  using Real = double;
  static constexpr size_t dim = ptc::CalFK::dimension;
};
