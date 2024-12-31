// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "ptcee/gaussian.h"
#include "ptcee/noisy_unit.h"
#include "ptcee/orientation_buffer.h"
#include "ptcee/scalar.h"
#include "ptcee/timestamp.h"
#include "ptcee/unit_projection_factor.h"

#include "Eigen/Core"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"

namespace ptc
{
namespace key
{
using gtsam::symbol_shorthand::A;
using gtsam::symbol_shorthand::K;
using gtsam::symbol_shorthand::P;
using gtsam::symbol_shorthand::Q;
using gtsam::symbol_shorthand::S;
using gtsam::symbol_shorthand::T;
using gtsam::symbol_shorthand::X;
}

struct PTZGraph
{
  gtsam::Values estimate;
  gtsam::NonlinearFactorGraph graph;
  OrientationBuffer orientation_buffer;

  PTZGraph();

  gtsam::Rot3 insertOrientationFactor(
    size_t frame_id,
    const NoisyTimestamp& timestamp
  );

  template<typename Cal>
  void insertProjectionFactor(
    size_t landmark_id,
    size_t frame_id,
    size_t prev_frame_id,
    double timestep,
    double sigma_timestep,
    const Gaussian<Eigen::Vector2d>& uv
  );

  template<typename Cal>
  void setCalibration(
    const Gaussian<Cal>& cal,
    bool fixate_k
  );

  void setDt(
    const Gaussian<double>& dt
  );

  void setLineDuration(
    const Gaussian<double>& line_duration,
    bool fixate
  );

  void setPTScale(
    const Eigen::Vector2d& pt_scale,
    const Eigen::Array2d& sigmas
  );

  template<typename Cal>
  Cal getCalEstimate() const;

  [[nodiscard]] std::map<size_t, Gaussian<gtsam::Unit3>> getLandmarkEstimates(
    const std::set<size_t>& landmark_ids
  ) const;

  [[nodiscard]] gtsam::Rot3 getOrientation(size_t frame_id) const;

  inline static const auto cal_key = key::K(0);
  inline static const auto dt_key = key::T(0);
  inline static const auto ld_key = key::T(1);
  inline static const auto pt_scale_key = key::S(0);
  inline static const auto pan_axis_key = key::A(0);
  inline static const auto tilt_axis_key = key::A(1);

  [[nodiscard]] static gtsam::Key getLandmarkKey(
    size_t id
  );

  [[nodiscard]] static gtsam::Key getPTKey(
    size_t id
  );
};

// ----- Implementation -----
template<typename Cal>
void PTZGraph::insertProjectionFactor(
  const size_t landmark_id,
  const size_t frame_id,
  const size_t prev_frame_id,
  const double timestep,
  const double sigma_timestep,
  const Gaussian<Eigen::Vector2d>& uv
)
{
  const auto landmark_key = getLandmarkKey(landmark_id);
  const auto pt_key = key::P(frame_id);
  const auto prev_pt_key = key::P(prev_frame_id);

  graph.push_back(
    UnitProjectionFactor<Cal>{
      landmark_key,
      pt_key,
      prev_pt_key,
      pan_axis_key,
      tilt_axis_key,
      cal_key,
      ld_key,
      timestep,
      sigma_timestep,
      uv
    }
  );
}

template<typename Cal>
void PTZGraph::setCalibration(
  const Gaussian<Cal>& cal,
  const bool fixate_k
)
{
  estimate.insert(cal_key, cal.x);

  if (fixate_k)
  {
    graph.template addPrior(
      cal_key,
      cal.x,
      gtsam::noiseModel::Gaussian::Covariance(cal.P)
    );
  }
}

template<typename Cal>
Cal PTZGraph::getCalEstimate() const
{
  return estimate.at<Cal>(cal_key);
}
}
