// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "ptcee/vector_traits.h"

#include "gtsam/geometry/Unit3.h"

template<>
struct ptc::vector_traits<gtsam::Unit3>
{
  using Real = double;
  static constexpr size_t dim = 2;
};
