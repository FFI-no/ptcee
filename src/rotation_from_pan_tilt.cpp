// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#include "ptcee/rotation_from_pan_tilt.h"

namespace
{
struct RotFromAngleAxis
{
  gtsam::Rot3 R;
  Eigen::Vector3d J_R_wrt_angle;
  Eigen::Matrix<double, 3, 2> J_R_wrt_axis;
};

[[nodiscard]] RotFromAngleAxis rotFromAngleAxis(
  double angle,
  const gtsam::Unit3& axis
);
}

namespace ptc
{
[[nodiscard]] Sophus::SO3d getSO3(
  const PanTilt& pan_tilt,
  const gtsam::Unit3& pan_axis,
  const gtsam::Unit3& tilt_axis
)
{
  const auto [pan, tilt] = pan_tilt.panAndTilt();

  return Sophus::SO3d{Eigen::AngleAxisd{pan, pan_axis.point3()}*Eigen::AngleAxisd{tilt, tilt_axis.point3()}};
}

RotFromPanTilt getRotation(
  const PanTilt& pan_tilt,
  const gtsam::Unit3& pan_axis,
  const gtsam::Unit3& tilt_axis
)
{
  const auto [pan, tilt] = pan_tilt.panAndTilt();
  const auto [rot_pan, J_rot_pan_wrt_pan, J_rot_pan_wrt_pan_axis] = rotFromAngleAxis(pan, pan_axis);
  const auto [rot_tilt, J_rot_tilt_wrt_tilt, J_rot_tilt_wrt_tilt_axis] = rotFromAngleAxis(tilt, tilt_axis);

  Eigen::Matrix3d J_rot_wrt_rot_pan;
  Eigen::Matrix3d J_rot_wrt_rot_tilt;
  const auto rot = rot_pan.compose(rot_tilt, J_rot_wrt_rot_pan, J_rot_wrt_rot_tilt);
  const auto J_rot_wrt_pan_tilt = (Eigen::Matrix<double, 3, 2>{}
    <<
    J_rot_wrt_rot_pan*J_rot_pan_wrt_pan, J_rot_wrt_rot_tilt*J_rot_tilt_wrt_tilt
  ).finished();
  const auto J_rot_wrt_pan_axis = J_rot_wrt_rot_pan*J_rot_pan_wrt_pan_axis;
  const auto J_rot_wrt_tilt_axis = J_rot_wrt_rot_tilt*J_rot_tilt_wrt_tilt_axis;

  return {
    rot,
    J_rot_wrt_pan_tilt,
    J_rot_wrt_pan_axis,
    J_rot_wrt_tilt_axis
  };
}
}

namespace
{
[[nodiscard]] RotFromAngleAxis rotFromAngleAxis(
  double angle,
  const gtsam::Unit3& axis
)
{
  Eigen::Matrix<double, 3, 2> J_u_wrt_axis;
  const auto u = axis.point3(J_u_wrt_axis);

  const auto rot = gtsam::Rot3::AxisAngle(axis, angle);

  const auto t_cross = Sophus::SO3d::hat(angle*u);
  const auto J_r = Eigen::Matrix3d::Identity() - (1 - std::cos(angle))/(angle*angle)*t_cross + (angle - std::sin(angle))/(angle*angle*angle)*t_cross*t_cross;
  const auto J_rot_wrt_u = J_r*angle;

  const auto& J_rot_wrt_angle = u;
  const auto J_rot_wrt_axis = J_rot_wrt_u*J_u_wrt_axis;

  return {rot, J_rot_wrt_angle, J_rot_wrt_axis};
}
}
