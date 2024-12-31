// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "ptcee/unit2.h"
#include "ptcee/vector_traits.h"

#include "Eigen/Core"

namespace ptc
{
class PanTilt
{
public:
  PanTilt() = default;

  PanTilt(
    Unit2&& pan,
    Unit2&& tilt
  );

  PanTilt(
    double pan_rad,
    double tilt_rad
  );

  /// \brief Constructs a PanTilt from (pan, tilt) in radians
  [[nodiscard]] static PanTilt fromAngles(
    const Eigen::Vector2d& angles
  );

  /// \brief Returns (pan, tilt) in radians
  /// \return
  [[nodiscard]] Eigen::Vector2d angles() const;

  struct PanAndTilt
  {
    double pan;
    double tilt;
  };

  [[nodiscard]] PanAndTilt panAndTilt() const;

  [[nodiscard]] const Unit2& panUnit() const;

  [[nodiscard]] const Unit2& tiltUnit() const;

private:
  Unit2 pan_;
  Unit2 tilt_;
};

struct NoisyPanTilt
{
  PanTilt x;
  Eigen::Matrix2d P;
};

[[nodiscard]] Eigen::Vector2d ominus(
  const PanTilt& v,
  const PanTilt& u
);

[[nodiscard]] PanTilt oplus(
  const PanTilt& u,
  const Eigen::Vector2d& xi
);

[[nodiscard]] PanTilt operator*(
  const Eigen::Array2d& scale,
  const PanTilt& pt
);

[[nodiscard]] PanTilt operator/(
  const PanTilt& pt,
  const Eigen::Array2d& scale
);

template<>
struct vector_traits<PanTilt>
{
  static constexpr int dim = 2;
  using Real = double;
};
}

namespace gtsam
{
template <typename T> struct traits; // fwd declare

template<> struct traits<ptc::PanTilt>
{
  [[nodiscard]] static constexpr size_t GetDimension(
    const ptc::PanTilt&
  )
  {
    return 2;
  }

  [[nodiscard]] static ptc::PanTilt Retract(
    const ptc::PanTilt& u,
    const Eigen::Vector2d& xi
  );

  [[nodiscard]] static Eigen::Vector2d Local(
    const ptc::PanTilt& v,
    const ptc::PanTilt& u
  );

  [[nodiscard]] static bool Equals(
    const ptc::PanTilt& lhs,
    const ptc::PanTilt& rhs,
    double tol
  );

  static void Print(
    const ptc::PanTilt&,
    const std::string&
  );
};
}
