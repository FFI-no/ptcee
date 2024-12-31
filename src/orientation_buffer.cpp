// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#include "ptcee/orientation_buffer.h"

namespace ptc
{
OrientationBuffer::TimeInterpolatedPanTilt OrientationBuffer::panTilt(
  const NoisyTimestamp& t
) const
{
  if (empty())
  {
    throw std::invalid_argument("Cannot interpolate on empty pt_buffer");
  }

  if (size() == 1)
  {
    return {
      begin()->second,
      Eigen::Vector2d::Zero()
    };
  }

  const auto lower = lower_bound({t, t});

  NoisyTimestamp next_dt;
  NoisyTimestamp next_t;
  NoisyTimestamp prev_t;
  NoisyPanTilt next_pt;
  NoisyPanTilt prev_pt;

  if (lower == end())
  {
    const auto next = rbegin();
    const auto prev = std::next(next);

    next_dt = next->first.dt;
    next_t = next->first.t;
    prev_t = prev->first.t;
    next_pt = next->second;
    prev_pt = prev->second;
  } else if (lower == begin())
  {
    const auto prev = lower;
    const auto next = std::next(prev);

    next_dt = next->first.dt;
    next_t = next->first.t;
    prev_t = prev->first.t;
    next_pt = next->second;
    prev_pt = prev->second;
  } else
  {
    const auto next = lower;
    const auto prev = std::prev(next);

    next_dt = next->first.dt;
    next_t = next->first.t;
    prev_t = prev->first.t;
    next_pt = next->second;
    prev_pt = prev->second;
  }

  const auto t0 = (prev_t.t + next_t.t - next_dt.t)/2;
  const auto l = (t.t - t0)/next_dt.t;
  const auto v = ominus(next_pt.x, prev_pt.x);
  const auto pt = oplus(prev_pt.x, l*v);

  const auto var_t = t.sigma_t*t.sigma_t;
  const auto var_prev_t = prev_t.sigma_t*prev_t.sigma_t;
  const auto var_next_t = next_t.sigma_t*next_t.sigma_t;
  const auto var_dt = next_dt.sigma_t*next_dt.sigma_t;

  const auto P_pt = (
    (1 - l)*(1 - l)*prev_pt.P
    + l*l*next_pt.P
    + (
        4*var_t
        + var_prev_t
        + var_next_t
        + (2*l - 1)*(2*l - 1)*var_dt
      )/(4*next_dt.t*next_dt.t)
    *v*v.transpose()
  ).eval();

  return {
    {
      pt,
      P_pt
    },
    ominus(next_pt.x, prev_pt.x)/next_dt.t
  };
}
}
