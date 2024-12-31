// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#include "ptcee/unit2.h"

#include <cmath>

namespace ptc
{
namespace
{
/// \brief wraps angle to [-pi, pi)
[[nodiscard]] double wrapAngle(
  double angle_rad
);
}

Unit2::Unit2()
  : phi_{0}
{}

Unit2::Unit2(
  const double phi
)
  : phi_{phi}
{}

Unit2 Unit2::fromAngle(
  const double phi
)
{
  return Unit2{wrapAngle(phi)};
}

double Unit2::angleRad() const
{
  return phi_;
}

Unit2 oplus(
  const Unit2& u,
  const double phi
)
{
  return Unit2::fromAngle(u.angleRad() + phi);
}

double ominus(
  const Unit2& v,
  const Unit2& u
)
{
  return wrapAngle(v.angleRad() - u.angleRad());
}

Unit2 operator*(
  const double s,
  const Unit2& u
)
{
  return Unit2::fromAngle(s*u.angleRad());
}

Unit2 operator*(
  const Unit2& u,
  const double s
)
{
  return s*u;
}

Unit2 operator/(
  const Unit2& u,
  const double s
)
{
  return (1/s)*u;
}

namespace
{
double wrapAngle(
  const double angle_rad
)
{
  constexpr auto two_pi = 2*M_PI;
  const auto v = angle_rad/two_pi;
  return two_pi*(v - std::floor(v + 0.5));
}
}
}
