// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

namespace ptc
{
using Timestamp = double;

struct NoisyTimestamp
{
  Timestamp t;
  double sigma_t;
};

[[nodiscard]] inline bool operator<(const NoisyTimestamp& lhs, const NoisyTimestamp& rhs)
{
  return lhs.t < rhs.t;
}
}
