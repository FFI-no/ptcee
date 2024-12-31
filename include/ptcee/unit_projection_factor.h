// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "ptcee/cal_fk.h"
#include "ptcee/gaussian.h"
#include "ptcee/rotation_from_pan_tilt.h"
#include "ptcee/scalar.h"

#include "gtsam/geometry/PinholeCamera.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

namespace ptc
{
template<typename Cal>
struct UnitProjectionFactor : public gtsam::NonlinearFactor
{
  struct Error
  {
    Eigen::Vector2d err;
    Eigen::Matrix2d P_err;
    Eigen::Matrix2d J_err_wrt_target_bearing;
    Eigen::Matrix2d J_err_wrt_pt;
    Eigen::Matrix2d J_err_wrt_prev_pt;
    Eigen::Matrix2d J_err_wrt_pan_axis;
    Eigen::Matrix2d J_err_wrt_tilt_axis;
    Eigen::Matrix<double, 2, Cal::dimension> J_err_wrt_cal;
    Eigen::Vector2d J_err_wrt_line_duration;
  };

  UnitProjectionFactor(
    const gtsam::Key& target_key,
    const gtsam::Key& pt_key,
    const gtsam::Key& prev_pt_key,
    const gtsam::Key& pan_axis_key,
    const gtsam::Key& tilt_axis_key,
    const gtsam::Key& cal_key,
    const gtsam::Key& ld_key,
    double timestep,
    double sigma_timestep,
    const Gaussian<Eigen::Vector2d>& measured_uv
  );

  [[nodiscard]] Error error(
    const gtsam::Unit3& target_bearing,
    const PanTilt& pt,
    const PanTilt& prev_pt,
    const gtsam::Unit3& pan_axis,
    const gtsam::Unit3& tilt_axis,
    const Cal& cal,
    double line_duration
  ) const;

  [[nodiscard]] double error(
    const gtsam::Values& x
  ) const override;

  [[nodiscard]] double pixelError(
    const gtsam::Values& x
  ) const;

  [[nodiscard]] double normPixelError(
    const gtsam::Values& x
  ) const;

  [[nodiscard]] double pixelError(
    const gtsam::Unit3& target_bearing,
    const PanTilt& pt,
    const PanTilt& prev_pt,
    const gtsam::Unit3& pan_axis,
    const gtsam::Unit3& tilt_axis,
    const Cal& cal,
    double line_duration
  ) const;

  [[nodiscard]] boost::shared_ptr<gtsam::GaussianFactor> linearize(
    const gtsam::Values& x
  ) const override;

  [[nodiscard]] size_t dim() const override
  {
    return 2;
  }

  [[nodiscard]] const gtsam::Key& getPtKey() const;

  [[nodiscard]] const gtsam::Key& getTargetKey() const;

  [[nodiscard]] const Gaussian<Eigen::Vector2d>& getMeasurement() const;

private:
  gtsam::Key target_key_;
  gtsam::Key pt_key_;
  gtsam::Key prev_pt_key_;
  gtsam::Key pan_axis_key_;
  gtsam::Key tilt_axis_key_;
  gtsam::Key cal_key_;
  gtsam::Key ld_key_;
  double timestep_;
  double sigma_timestep_;
  Gaussian<Eigen::Vector2d> measured_uv_;
};

// ----- Implementation -----
template<typename Cal>
UnitProjectionFactor<Cal>::UnitProjectionFactor(
  const gtsam::Key& target_key,
  const gtsam::Key& pt_key,
  const gtsam::Key& prev_pt_key,
  const gtsam::Key& pan_axis_key,
  const gtsam::Key& tilt_axis_key,
  const gtsam::Key& cal_key,
  const gtsam::Key& ld_key,
  const double timestep,
  const double sigma_timestep,
  const Gaussian<Eigen::Vector2d>& measured_uv
)
  : gtsam::NonlinearFactor{
  std::vector{
    target_key,
    pt_key,
    prev_pt_key,
    pan_axis_key,
    tilt_axis_key,
    cal_key,
    ld_key
  }}
  , target_key_{target_key}
  , pt_key_{pt_key}
  , prev_pt_key_{prev_pt_key}
  , pan_axis_key_{pan_axis_key}
  , tilt_axis_key_{tilt_axis_key}
  , cal_key_{cal_key}
  , ld_key_{ld_key}
  , timestep_{timestep}
  , sigma_timestep_{sigma_timestep}
  , measured_uv_{measured_uv}
{}

template<typename Cal>
typename UnitProjectionFactor<Cal>::Error UnitProjectionFactor<Cal>::error(
  const gtsam::Unit3& target_bearing,
  const PanTilt& pt,
  const PanTilt& prev_pt,
  const gtsam::Unit3& pan_axis,
  const gtsam::Unit3& tilt_axis,
  const Cal& cal,
  const double line_duration
) const
{
  const auto y = measured_uv_.x.y();
  const auto line_t = y*line_duration;
  const auto J_line_t_wrt_line_duration = y;

  const auto l = line_t/timestep_;
  const auto J_l_wrt_timestep = -line_t/(timestep_*timestep_);
  const auto J_l_wrt_line_duration = 1/timestep_*J_line_t_wrt_line_duration;

  const auto cur_pt = oplus(pt, -l*ominus(prev_pt, pt));
  const auto J_cur_pt_wrt_pt = (l + 1)*Eigen::Matrix2d::Identity();
  const auto J_cur_pt_wrt_prev_pt = -l*Eigen::Matrix2d::Identity();
  const auto J_cur_pt_wrt_l = -ominus(prev_pt, pt);

  const auto [rot, J_rot_wrt_cur_pt, J_rot_wrt_pan_axis, J_rot_wrt_tilt_axis] = getRotation(cur_pt, pan_axis, tilt_axis);

  const auto J_rot_wrt_pt = (J_rot_wrt_cur_pt*J_cur_pt_wrt_pt).eval();
  const auto J_rot_wrt_prev_pt = (J_rot_wrt_cur_pt*J_cur_pt_wrt_prev_pt).eval();
  const auto J_rot_wrt_timestep = J_rot_wrt_cur_pt*J_cur_pt_wrt_l*J_l_wrt_timestep;
  const auto J_rot_wrt_line_duration = J_rot_wrt_cur_pt*J_cur_pt_wrt_l*J_l_wrt_line_duration;

  static thread_local const gtsam::Rot3 R_FRD_from_RDF{
    (Eigen::Matrix3d{}
      << Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()
    ).finished()
  };

  Eigen::Matrix3d J_rot_rdf_wrt_rot;
  const auto rot_rdf = rot.compose(R_FRD_from_RDF, J_rot_rdf_wrt_rot);

  const gtsam::Pose3 pose = {
    rot_rdf,
    Eigen::Vector3d::Zero()
  };

  const gtsam::PinholeCamera camera{
    pose,
    cal
  };

  Eigen::Matrix<double, 2, gtsam::Pose3::dimension> J_predicted_uv_wrt_pose;
  Eigen::Matrix2d J_predicted_uv_wrt_target_bearing;
  Eigen::Matrix<double, 2, Cal::dimension> J_predicted_uv_wrt_cal;

  const auto predicted_uv = camera.project(
    target_bearing,
    J_predicted_uv_wrt_pose,
    J_predicted_uv_wrt_target_bearing,
    J_predicted_uv_wrt_cal
  );

  static thread_local const gtsam::Matrix J_err_wrt_predicted_uv = gtsam::Matrix::Identity(2, 2);
  static thread_local const gtsam::Matrix J_err_wrt_measured_uv = -gtsam::Matrix::Identity(2, 2);
  const auto err = predicted_uv - measured_uv_.x;

  const Eigen::Matrix2d J_err_wrt_target_bearing = (J_err_wrt_predicted_uv * J_predicted_uv_wrt_target_bearing).eval();

  const auto J_predicted_uv_wrt_rot_rdf = J_predicted_uv_wrt_pose.topLeftCorner<2, 3>().eval();
  const auto J_err_wrt_rot = (J_err_wrt_predicted_uv * J_predicted_uv_wrt_rot_rdf * J_rot_rdf_wrt_rot).eval();

  const Eigen::Matrix2d J_err_wrt_pt = J_err_wrt_rot*J_rot_wrt_pt;

  const Eigen::Matrix2d J_err_wrt_prev_pt = J_err_wrt_rot*J_rot_wrt_prev_pt;

  const Eigen::Matrix2d J_err_wrt_pan_axis = J_err_wrt_rot*J_rot_wrt_pan_axis;
  const Eigen::Matrix2d J_err_wrt_tilt_axis = J_err_wrt_rot*J_rot_wrt_tilt_axis;

  const auto J_err_wrt_cal = (J_err_wrt_predicted_uv * J_predicted_uv_wrt_cal).eval();

  const Eigen::Vector2d J_err_wrt_line_duration = (J_err_wrt_rot * J_rot_wrt_line_duration).eval();

  const Eigen::Vector2d J_err_wrt_timestep = (J_err_wrt_rot * J_rot_wrt_timestep).eval();

  const auto P_timestep = sigma_timestep_*sigma_timestep_;
  const Eigen::Matrix2d P_err = J_err_wrt_measured_uv*measured_uv_.P*J_err_wrt_measured_uv.transpose()
    + J_err_wrt_timestep*P_timestep*J_err_wrt_timestep.transpose();

  return {
    err,
    P_err,
    J_err_wrt_target_bearing,
    J_err_wrt_pt,
    J_err_wrt_prev_pt,
    J_err_wrt_pan_axis,
    J_err_wrt_tilt_axis,
    J_err_wrt_cal,
    J_err_wrt_line_duration
  };
}

template<typename Cal>
double UnitProjectionFactor<Cal>::error(
  const gtsam::Values& x
) const
{
  const auto target_bearing = x.at<gtsam::Unit3>(target_key_);
  const auto pt = x.at<PanTilt>(pt_key_);
  const auto prev_pt = x.at<PanTilt>(prev_pt_key_);
  const auto pan_axis = x.at<gtsam::Unit3>(pan_axis_key_);
  const auto tilt_axis = x.at<gtsam::Unit3>(tilt_axis_key_);
  const auto cal = x.at<Cal>(cal_key_);
  const double line_duration = x.at<ptc::PositiveScalar>(ld_key_);

  const auto full_error = error(target_bearing, pt, prev_pt, pan_axis, tilt_axis, cal, line_duration);

  const gtsam::noiseModel::Robust huber(
    gtsam::noiseModel::mEstimator::Huber::Create(0.75),
    gtsam::noiseModel::Gaussian::Covariance(full_error.P_err)
  );

  return huber.loss(huber.squaredMahalanobisDistance(full_error.err));
}

template<typename Cal>
double UnitProjectionFactor<Cal>::pixelError(
  const gtsam::Values& x
) const
{
  const auto target_bearing = x.at<gtsam::Unit3>(target_key_);
  const auto pt = x.at<PanTilt>(pt_key_);
  const auto prev_pt = x.at<PanTilt>(prev_pt_key_);
  const auto pan_axis = x.at<gtsam::Unit3>(pan_axis_key_);
  const auto tilt_axis = x.at<gtsam::Unit3>(tilt_axis_key_);
  const auto cal = x.at<Cal>(cal_key_);
  const double line_duration = x.at<ptc::PositiveScalar>(ld_key_);

  return pixelError(target_bearing, pt, prev_pt, pan_axis, tilt_axis, cal, line_duration);
}

template<typename Cal>
double UnitProjectionFactor<Cal>::normPixelError(
  const gtsam::Values& x
) const
{
  return pixelError(x)/measured_uv_.P.diagonal().norm();
}

template<typename Cal>
double UnitProjectionFactor<Cal>::pixelError(
  const gtsam::Unit3& target_bearing,
  const PanTilt& pt,
  const PanTilt& prev_pt,
  const gtsam::Unit3& pan_axis,
  const gtsam::Unit3& tilt_axis,
  const Cal& cal,
  const double line_duration
) const
{
  const auto y = measured_uv_.x.y();
  const auto line_t = y*line_duration;
  const auto l = line_t/timestep_;
  const auto cur_pt = oplus(pt, -l*ominus(prev_pt, pt));
  const auto rot = getRotation(cur_pt, pan_axis, tilt_axis).rot;

  static thread_local const gtsam::Rot3 R_FRD_from_RDF{
    (Eigen::Matrix3d{}
      << Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()
    ).finished()
  };

  const auto rot_rdf = rot.compose(R_FRD_from_RDF);

  const gtsam::Pose3 pose = {
    rot_rdf,
    Eigen::Vector3d::Zero()
  };

  const gtsam::PinholeCamera camera{
    pose,
    cal
  };

  const auto predicted_uv = camera.project(target_bearing);

  return (predicted_uv - measured_uv_.x).norm();
}

template<typename Cal>
boost::shared_ptr<gtsam::GaussianFactor> UnitProjectionFactor<Cal>::linearize(
  const gtsam::Values& x
) const
{
  const auto target_bearing = x.at<gtsam::Unit3>(target_key_);
  const auto pt = x.at<PanTilt>(pt_key_);
  const auto prev_pt = x.at<PanTilt>(prev_pt_key_);
  const auto pan_axis = x.at<gtsam::Unit3>(pan_axis_key_);
  const auto tilt_axis = x.at<gtsam::Unit3>(tilt_axis_key_);
  const auto cal = x.at<Cal>(cal_key_);
  const double line_duration = x.at<ptc::PositiveScalar>(ld_key_);

  const auto res = error(target_bearing, pt, prev_pt, pan_axis, tilt_axis, cal, line_duration);

  std::vector<gtsam::Matrix> A{
    res.J_err_wrt_target_bearing,
    res.J_err_wrt_pt,
    res.J_err_wrt_prev_pt,
    res.J_err_wrt_pan_axis,
    res.J_err_wrt_tilt_axis,
    res.J_err_wrt_cal,
    res.J_err_wrt_line_duration
  };
  gtsam::Vector b = -res.err;

  const gtsam::noiseModel::Robust huber(
    gtsam::noiseModel::mEstimator::Huber::Create(0.75),
    gtsam::noiseModel::Gaussian::Covariance(res.P_err)
  );
  huber.WhitenSystem(A, b);

  return boost::make_shared<gtsam::JacobianFactor>(
    std::vector{
      std::pair{target_key_, std::move(A[0])},
      std::pair{pt_key_, std::move(A[1])},
      std::pair{prev_pt_key_, std::move(A[2])},
      std::pair{pan_axis_key_, std::move(A[3])},
      std::pair{tilt_axis_key_, std::move(A[4])},
      std::pair{cal_key_, std::move(A[5])},
      std::pair{ld_key_, std::move(A[6])}
    },
    b
  );
}

template<typename Cal>
const gtsam::Key& UnitProjectionFactor<Cal>::getPtKey() const
{
  return pt_key_;
}

template<typename Cal>
const gtsam::Key& UnitProjectionFactor<Cal>::getTargetKey() const
{
  return target_key_;
}

template<typename Cal>
const Gaussian<Eigen::Vector2d>& UnitProjectionFactor<Cal>::getMeasurement() const
{
  return measured_uv_;
}
}
