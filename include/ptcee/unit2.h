// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

namespace ptc
{
class Unit2
{
public:
  Unit2();

  static Unit2 fromAngle(
    double phi
  );

  Unit2(const Unit2&) = default;
  Unit2(Unit2&&) = default;
  Unit2& operator=(const Unit2&) = default;
  Unit2& operator=(Unit2&&) = default;

  [[nodiscard]] double angleRad() const;

private:
  double phi_;

  /// \param phi angle in rad normalized to [-pi, pi)
  explicit Unit2(
    double phi
  );
};

/// \brief returns v = u oplus phi
/// \param u
/// \param phi
/// Jacobians are trivial:
///   J_v_wrt_u = 1
///   J_v_wrt_phi = 1
/// \return
[[nodiscard]] Unit2 oplus(
  const Unit2& u,
  double phi
);

/// \brief Computes phi = v ominus u
/// \param v
/// \param u
/// Jacobians are trivial:
///   J_phi_wrt_v = 1
///   J_phi_wrt_u = -1
/// \return
[[nodiscard]] double ominus(
  const Unit2& v,
  const Unit2& u
);

[[nodiscard]] Unit2 operator*(
  double s,
  const Unit2& u
);

[[nodiscard]] Unit2 operator*(
  const Unit2& u,
  double s
);

[[nodiscard]] Unit2 operator/(
  const Unit2& u,
  double s
);
}
