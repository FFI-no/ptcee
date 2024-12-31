// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#include "ptcee/ptz_graph.h"

#include "ptcee/rotation_from_pan_tilt.h"
#include "ptcee/pt_buffer_factor.h"

#include "gtsam/nonlinear/Marginals.h"

namespace ptc
{
PTZGraph::PTZGraph()
{
  {
    const gtsam::Unit3 prior_pan_axis{Eigen::Vector3d::UnitZ()};

    estimate.insert(pan_axis_key, prior_pan_axis);
    graph.addPrior(
      pan_axis_key,
      prior_pan_axis,
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Vector2d{5e-2, 5e-2})
    );
  }

  {
    const gtsam::Unit3 prior_tilt_axis{Eigen::Vector3d::UnitY()};

    estimate.insert(tilt_axis_key, prior_tilt_axis);
    graph.addPrior(
      tilt_axis_key,
      prior_tilt_axis,
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Vector2d{5e-2, 5e-2})
    );
  }
}

gtsam::Rot3 PTZGraph::insertOrientationFactor(
  const size_t frame_id,
  const NoisyTimestamp& timestamp
)
{
  const auto pt_key = getPTKey(frame_id);

  const auto dt = estimate.at<Scalar>(dt_key)(0);
  const auto pt = orientation_buffer.panTilt({timestamp.t + dt, timestamp.sigma_t}).pt.x;
  estimate.insert(pt_key, pt);

  graph.push_back(
    PTBufferFactor(
      pt_key,
      pt_scale_key,
      dt_key,
      orientation_buffer,
      timestamp
    )
  );

  const auto pan_axis = estimate.at<gtsam::Unit3>(pan_axis_key);
  const auto tilt_axis = estimate.at<gtsam::Unit3>(tilt_axis_key);

  return getRotation(pt, pan_axis, tilt_axis).rot;
}

void PTZGraph::setDt(
  const Gaussian<double>& dt
)
{
  estimate.insert(
    dt_key,
    Scalar{dt.x}
  );
}

void PTZGraph::setLineDuration(
  const Gaussian<double>& line_duration,
  const bool fixate
)
{
  estimate.insert(
    ld_key,
    PositiveScalar{line_duration.x}
  );

  if (fixate)
  {
    graph.addPrior(
      ld_key,
      PositiveScalar{line_duration.x},
      gtsam::noiseModel::Constrained::All(1)
    );
  }
}

void PTZGraph::setPTScale(
  const Eigen::Vector2d& pt_scale,
  const Eigen::Array2d& sigmas
)
{
  estimate.insert(
    pt_scale_key,
    pt_scale
  );

  graph.addPrior(
    pt_scale_key,
    pt_scale,
    gtsam::noiseModel::Diagonal::Sigmas(sigmas)
  );
}

std::map<size_t, Gaussian<gtsam::Unit3>> PTZGraph::getLandmarkEstimates(
  const std::set<size_t>& landmark_ids
) const
{
  std::map<size_t, Gaussian<gtsam::Unit3>> landmarks;
  gtsam::Marginals marginals{graph, estimate};

  for (const auto landmark_id : landmark_ids)
  {
    const auto landmark_key = getLandmarkKey(landmark_id);

    landmarks[landmark_id] = {
      estimate.at<gtsam::Unit3>(landmark_key),
      marginals.marginalCovariance(landmark_key)
    };
  }

  return landmarks;
}

gtsam::Rot3 PTZGraph::getOrientation(
  const size_t frame_id
) const
{
  const auto pt_key = getPTKey(frame_id);

  const auto pt = estimate.at<PanTilt>(pt_key);
  const auto pan_axis = estimate.at<gtsam::Unit3>(pan_axis_key);
  const auto tilt_axis = estimate.at<gtsam::Unit3>(tilt_axis_key);

  return getRotation(pt, pan_axis, tilt_axis).rot;
}


gtsam::Key PTZGraph::getLandmarkKey(
  const size_t id
)
{
  return key::X(id);
}

gtsam::Key PTZGraph::getPTKey(
  const size_t id
)
{
  return key::P(id);
}
}
