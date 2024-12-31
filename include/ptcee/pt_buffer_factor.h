// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "ptcee/orientation_buffer.h"
#include "ptcee/timestamp.h"

#include "gtsam/geometry/Rot3.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

#include <map>

namespace ptc
{
struct PTBufferFactor : public gtsam::NonlinearFactor
{
  struct Error
  {
    Eigen::Vector2d err;
    Eigen::Matrix2d P_err;
    Eigen::Matrix2d J_err_wrt_pt;
    Eigen::Matrix2d J_err_wrt_pt_scale;
    Eigen::Vector2d J_err_wrt_dt;
  };

  PTBufferFactor(
    const gtsam::Key& pt_key,
    const gtsam::Key& pt_scale_key,
    const gtsam::Key& dt_key,
    const OrientationBuffer& orientation_buffer,
    const NoisyTimestamp& t
  );

  [[nodiscard]] Error error(
    const PanTilt& pt,
    const Eigen::Vector2d& pt_scale,
    double dt
  ) const;

  [[nodiscard]] double error(
    const gtsam::Values& x
  ) const override;

  [[nodiscard]] Eigen::Vector2d panTiltError(
    const gtsam::Values& x
  ) const;

  [[nodiscard]] boost::shared_ptr<gtsam::GaussianFactor> linearize(
    const gtsam::Values& x
  ) const override;

  [[nodiscard]] size_t dim() const override
  {
    return 2;
  }

private:
  gtsam::Key pt_key_;
  gtsam::Key pt_scale_key_;
  gtsam::Key dt_key_;
  const OrientationBuffer& orientation_buffer_;
  NoisyTimestamp t_;
};
}
