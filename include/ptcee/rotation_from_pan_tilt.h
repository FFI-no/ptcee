// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "ptcee/pan_tilt.h"
#include "ptcee/timestamp.h"

#include "Eigen/Core"
#include "gtsam/geometry/Rot3.h"
#include "sophus/so3.hpp"

namespace ptc
{
struct RotFromPanTilt
{
  gtsam::Rot3 rot;
  Eigen::Matrix<double, 3, 2> J_rot_wrt_pan_tilt;
  Eigen::Matrix<double, 3, 2> J_rot_wrt_pan_axis;
  Eigen::Matrix<double, 3, 2> J_rot_wrt_tilt_axis;
};

[[nodiscard]] Sophus::SO3d getSO3(
  const PanTilt& pan_tilt,
  const gtsam::Unit3& pan_axis,
  const gtsam::Unit3& tilt_axis
);

[[nodiscard]] RotFromPanTilt getRotation(
  const PanTilt& pan_tilt,
  const gtsam::Unit3& pan_axis,
  const gtsam::Unit3& tilt_axis
);
}
