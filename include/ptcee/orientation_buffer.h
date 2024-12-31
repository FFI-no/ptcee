// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "ptcee/pan_tilt.h"
#include "ptcee/timestamp.h"

#include "Eigen/Core"
#include "gtsam/geometry/Rot3.h"

#include <map>

namespace ptc
{

struct NoisyDeltaTimestamp
{
  NoisyTimestamp t;
  NoisyTimestamp dt;
};

[[nodiscard]] inline bool operator<(
  const NoisyDeltaTimestamp& lhs,
  const NoisyDeltaTimestamp& rhs
)
{
  return lhs.t < rhs.t;
}

class OrientationBuffer : public std::map<NoisyDeltaTimestamp, NoisyPanTilt>
{
public:
  struct TimeInterpolatedPanTilt
  {
    NoisyPanTilt pt;
    Eigen::Vector2d J_pt_wrt_t;
  };

  struct NoisyRot
  {
    gtsam::Rot3 x;
    Eigen::Matrix3d P;
  };

  struct TimeInterpolatedRot
  {
    NoisyRot rot;
    Eigen::Vector3d J_rot_wrt_t;
  };

  template<typename... Args>
  explicit OrientationBuffer(Args&&... args)
    : std::map<NoisyDeltaTimestamp, NoisyPanTilt>{std::forward<Args>(args)...}
  {}

  [[nodiscard]] TimeInterpolatedPanTilt panTilt(
    const NoisyTimestamp& t
  ) const;
};
}
