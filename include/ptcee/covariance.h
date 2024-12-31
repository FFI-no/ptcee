// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "Eigen/Core"

namespace ptc
{
template<int dim, typename Real>
using Covariance = Eigen::Matrix<Real, dim, dim>;
}
