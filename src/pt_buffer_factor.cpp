// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#include "ptcee/pt_buffer_factor.h"

#include "ptcee/scalar.h"

namespace ptc
{
PTBufferFactor::PTBufferFactor(
  const gtsam::Key& pt_key,
  const gtsam::Key& pt_scale_key,
  const gtsam::Key& dt_key,
  const OrientationBuffer& orientation_buffer,
  const NoisyTimestamp& t
)
  : gtsam::NonlinearFactor{std::vector{pt_key, dt_key}}
  , pt_key_{pt_key}
  , pt_scale_key_{pt_scale_key}
  , dt_key_{dt_key}
  , orientation_buffer_{orientation_buffer}
  , t_{t}
{}

PTBufferFactor::Error PTBufferFactor::error(
  const PanTilt& pt,
  const Eigen::Vector2d& pt_scale,
  const double dt
) const
{
  const auto [measured_pt, J_measured_pt_wrt_dt] = orientation_buffer_.panTilt({t_.t + dt, t_.sigma_t});

  const auto err = ominus(pt_scale.array()*pt, measured_pt.x);
  const Eigen::Matrix2d J_err_wrt_pt = pt_scale.asDiagonal();
  const Eigen::Matrix2d J_err_wrt_pt_scale = pt.angles().asDiagonal();
  const Eigen::Matrix2d J_err_wrt_measured_pt = -Eigen::Matrix2d::Identity();

  const auto P_err = J_err_wrt_measured_pt*measured_pt.P*J_err_wrt_measured_pt.transpose();

  const auto J_err_wrt_dt = (J_err_wrt_measured_pt * J_measured_pt_wrt_dt).eval();

  return {
    err,
    P_err,
    J_err_wrt_pt,
    J_err_wrt_pt_scale,
    J_err_wrt_dt
  };
}

double PTBufferFactor::error(
  const gtsam::Values& x
) const
{
  const auto pt = x.at<PanTilt>(pt_key_);
  const auto pt_scale = x.at<Eigen::Vector2d>(pt_scale_key_);
  const double dt = x.at<ptc::Scalar>(dt_key_);

  const auto full_error = error(pt, pt_scale, dt);
  const auto& err = full_error.err;
  const auto& P_err = full_error.P_err;

  const gtsam::noiseModel::Robust huber(
    gtsam::noiseModel::mEstimator::Huber::Create(1.345),
    gtsam::noiseModel::Gaussian::Covariance(P_err)
  );

  return huber.loss(huber.squaredMahalanobisDistance(err));
}

Eigen::Vector2d PTBufferFactor::panTiltError(
  const gtsam::Values& x
) const
{
  const auto pt = x.at<PanTilt>(pt_key_);
  const auto pt_scale = x.at<Eigen::Vector2d>(pt_scale_key_);
  const double dt = x.at<ptc::Scalar>(dt_key_);

  const auto measured_pt = orientation_buffer_.panTilt({t_.t + dt, t_.sigma_t}).pt.x;

  return ominus(pt_scale.array()*pt, measured_pt);
}

boost::shared_ptr<gtsam::GaussianFactor> PTBufferFactor::linearize(
  const gtsam::Values &x
) const
{
  if (!active(x))
  {
    return boost::shared_ptr<gtsam::JacobianFactor>();
  }

  const auto pt = x.at<PanTilt>(pt_key_);
  const auto pt_scale = x.at<Eigen::Vector2d>(pt_scale_key_);
  const double dt = x.at<ptc::Scalar>(dt_key_);

  const auto [err, P_err, J_err_wrt_pt, J_err_wrt_pt_scale, J_err_wrt_dt] = error(pt, pt_scale, dt);

  std::vector<gtsam::Matrix> A{
    J_err_wrt_pt,
    J_err_wrt_pt_scale,
    J_err_wrt_dt
  };
  gtsam::Vector b = -err;

  const gtsam::noiseModel::Robust huber(
    gtsam::noiseModel::mEstimator::Huber::Create(1.345),
    gtsam::noiseModel::Gaussian::Covariance(P_err)
  );
  huber.WhitenSystem(A, b);

  return boost::make_shared<gtsam::JacobianFactor>(
    std::vector{
      std::pair{pt_key_, std::move(A[0])},
      std::pair{pt_scale_key_, std::move(A[1])},
      std::pair{dt_key_, std::move(A[2])}
    },
    b
  );
}
}
