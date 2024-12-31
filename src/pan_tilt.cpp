// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#include "ptcee/pan_tilt.h"

namespace ptc
{
PanTilt::PanTilt(
  Unit2&& pan,
  Unit2&& tilt
)
  : pan_{std::move(pan)}
  , tilt_{std::move(tilt)}
{}

PanTilt::PanTilt(
  const double pan_rad,
  const double tilt_rad
)
  : pan_{Unit2::fromAngle(pan_rad)}
  , tilt_{Unit2::fromAngle(tilt_rad)}
{}

PanTilt PanTilt::fromAngles(
  const Eigen::Vector2d& angles
)
{
  return {angles.x(), angles.y()};
}

Eigen::Vector2d PanTilt::angles() const
{
  return Eigen::Vector2d{
    pan_.angleRad(),
    tilt_.angleRad()
  };
}

PanTilt::PanAndTilt PanTilt::panAndTilt() const
{
  return {
    pan_.angleRad(),
    tilt_.angleRad()
  };
}

const Unit2& PanTilt::panUnit() const
{
  return pan_;
}

const Unit2& PanTilt::tiltUnit() const
{
  return tilt_;
}

Eigen::Vector2d ominus(
  const PanTilt& v,
  const PanTilt& u
)
{
  return Eigen::Vector2d{
    ominus(v.panUnit(), u.panUnit()),
    ominus(v.tiltUnit(), u.tiltUnit())
  };
}

PanTilt oplus(
  const PanTilt& u,
  const Eigen::Vector2d& xi
)
{
  return {
    oplus(u.panUnit(), xi.x()),
    oplus(u.tiltUnit(), xi.y())
  };
}

PanTilt operator*(
  const Eigen::Array2d& scale,
  const PanTilt& pt
)
{
  return {
    scale.x()*pt.panUnit(),
    scale.y()*pt.tiltUnit()
  };
}

PanTilt operator/(
  const PanTilt& pt,
  const Eigen::Array2d& scale
)
{
  return {
    pt.panUnit()/scale.x(),
    pt.tiltUnit()/scale.y()
  };
}
}

namespace gtsam
{
ptc::PanTilt traits<ptc::PanTilt>::Retract(
  const ptc::PanTilt& u,
  const Eigen::Vector2d& xi
)
{
  return ptc::oplus(u, xi);
}

Eigen::Vector2d traits<ptc::PanTilt>::Local(
  const ptc::PanTilt& v,
  const ptc::PanTilt& u
)
{
  return ptc::ominus(v, u);
}

bool traits<ptc::PanTilt>::Equals(
  const ptc::PanTilt&,
  const ptc::PanTilt&, const double
)
{
  throw std::runtime_error{"not implemented"};
}

void traits<ptc::PanTilt>::Print(
  const ptc::PanTilt&,
  const std::string&
)
{
  throw std::runtime_error{"not implemented"};
}
}
