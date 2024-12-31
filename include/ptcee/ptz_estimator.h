// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "ptcee/gaussian.h"
#include "ptcee/noisy_unit.h"
#include "ptcee/ptz_graph.h"
#include "ptcee/timestamp.h"

#include "Eigen/Core"
#include "gtsam/geometry/PinholeCamera.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"

#include <map>

namespace ptc
{
template<typename Cal>
class PTZEstimator
{
public:
  using FrameId = size_t;

  struct NoisyRot
  {
    gtsam::Rot3 rot;
    Eigen::Matrix3d P;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct Estimate
  {
    Gaussian<Cal> cal;
    Gaussian<double> dt;
    Gaussian<gtsam::Unit3> pan_axis;
    Gaussian<gtsam::Unit3> tilt_axis;
    Gaussian<double> ld;
    Gaussian<Eigen::Vector2d> pt_scale;
    std::map<FrameId, PanTilt> pts;
    std::map<size_t, Gaussian<gtsam::Unit3>> updated_landmarks;
    gtsam::JointMarginal marginal;
  };

 PTZEstimator(
    const Gaussian<Cal>& initial_cal,
    const Gaussian<double>& initial_dt,
    const Gaussian<double>& initial_line_duration,
    const Eigen::Vector2d& initial_pt_scale,
    const Eigen::Array2d& pt_scale_sigmas,
    bool fixate_k = false,
    bool fixate_ld = false
  );

  void insertPanTilt(
    const NoisyDeltaTimestamp& timestamp,
    const NoisyPanTilt& pan_tilt
  );

  FrameId insertObservations(
    const NoisyDeltaTimestamp& timestamp,
    const std::map<size_t, Gaussian<Eigen::Vector2d>>& observations
  );

  void estimate();

  [[nodiscard]] Estimate getEstimate() const;

  [[nodiscard]] const gtsam::Values& getRawEstimate() const
  {
    return graph_.estimate;
  }

  [[nodiscard]] const gtsam::NonlinearFactorGraph& getGraph() const
  {
    return graph_.graph;
  }

  [[nodiscard]] const OrientationBuffer& getBuffer() const
  {
    return graph_.orientation_buffer;
  }

  [[nodiscard]] PanTilt getPanTilt(const Timestamp& t) const
  {
    return graph_.orientation_buffer.panTilt({t, 0}).pt.x;
  }

  struct RotAndTwist
  {
    gtsam::Rot3 R_base_from_cam;
    Eigen::Vector3d twist;
  };

  [[nodiscard]] RotAndTwist getRotAndTwist(
    const Timestamp& t,
    const gtsam::Unit3& pan_axis,
    const gtsam::Unit3& tilt_axis,
    const Eigen::Array2d& pt_scale
  ) const
  {
    const auto [pt, J_pt_wrt_t] = graph_.orientation_buffer.panTilt({t, 0});
    const auto R_b_from_c_frd = getRotation(pt.x/pt_scale, pan_axis, tilt_axis);

    static thread_local const gtsam::Rot3 R_FRD_from_RDF{
      (Eigen::Matrix3d{}
        << Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()
      ).finished()
    };

    Eigen::Matrix3d J_rr;
    const auto R_b_from_c = R_b_from_c_frd.rot.compose(R_FRD_from_RDF, J_rr);

    return {
      R_b_from_c,
      J_rr*R_b_from_c_frd.J_rot_wrt_pan_tilt*(1/pt_scale).matrix().asDiagonal()*J_pt_wrt_t
    };
  }

private:
  using Camera = gtsam::PinholeCamera<Cal>;

  PTZGraph graph_;
  std::optional<FrameId> prev_frame_id_;
  std::set<size_t> landmark_ids_;
  std::set<FrameId> frame_ids_;

  size_t rot_counter_;

  FrameId createFrameId();
};

// ----- Implementation -----
template<typename Cal>
PTZEstimator<Cal>::PTZEstimator(
  const Gaussian<Cal>& initial_cal,
  const Gaussian<double>& initial_dt,
  const Gaussian<double>& initial_line_duration,
  const Eigen::Vector2d& initial_pt_scale,
  const Eigen::Array2d& pt_scale_sigmas,
  const bool fixate_k,
  const bool fixate_ld
)
  : rot_counter_{0}
{
  graph_.setCalibration(initial_cal, fixate_k);
  graph_.setDt(initial_dt);
  graph_.setLineDuration(initial_line_duration, fixate_ld);
  graph_.setPTScale(initial_pt_scale, pt_scale_sigmas);
}

template<typename Cal>
void PTZEstimator<Cal>::insertPanTilt(
  const NoisyDeltaTimestamp& timestamp,
  const NoisyPanTilt& pan_tilt
)
{
  graph_.orientation_buffer[timestamp] = pan_tilt;
}

template<typename Cal>
void PTZEstimator<Cal>::estimate()
{
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_.graph, graph_.estimate);

  optimizer.optimize();

  graph_.estimate = optimizer.values();
}

template<typename Cal>
typename PTZEstimator<Cal>::Estimate PTZEstimator<Cal>::getEstimate() const
{
  const gtsam::Marginals marginals(graph_.graph, graph_.estimate);
  const auto joint_marginal = marginals.jointMarginalCovariance(
    std::vector{
      PTZGraph::cal_key,
      PTZGraph::dt_key,
      PTZGraph::ld_key,
      PTZGraph::pt_scale_key
    }
  );

  const Gaussian<gtsam::Unit3> pan_axis = {
    graph_.estimate.template at<gtsam::Unit3>(PTZGraph::pan_axis_key),
    marginals.marginalCovariance(PTZGraph::pan_axis_key)
  };
  const Gaussian<gtsam::Unit3> tilt_axis = {
    graph_.estimate.template at<gtsam::Unit3>(PTZGraph::tilt_axis_key),
    marginals.marginalCovariance(PTZGraph::tilt_axis_key)
  };

  std::map<size_t, Gaussian<gtsam::Unit3>> updated_landmarks;

  {
    for (const auto& landmark_id : landmark_ids_)
    {
      const auto landmark_key = PTZGraph::getLandmarkKey(landmark_id);

      updated_landmarks[landmark_id] = {
        graph_.estimate.template at<gtsam::Unit3>(landmark_key),
        2e-8*Eigen::Matrix2d::Identity()
      };
    }
  }

  std::map<size_t, PanTilt> pts;

  {
    for (const auto& frame_id : frame_ids_)
    {
      const auto pt_key = PTZGraph::getPTKey(frame_id);

      pts[frame_id] = graph_.estimate.template at<PanTilt>(pt_key);
    }
  }

  const Gaussian<Cal> cal = {
    graph_.estimate.template at<Cal>(PTZGraph::cal_key),
    joint_marginal.at(PTZGraph::cal_key, PTZGraph::cal_key)
  };

  const Gaussian<double> dt = {
    graph_.estimate.template at<Scalar>(PTZGraph::dt_key),
    joint_marginal.at(PTZGraph::dt_key, PTZGraph::dt_key)
  };
  const Gaussian<double> line_duration = {
    graph_.estimate.template at<PositiveScalar>(PTZGraph::ld_key),
    joint_marginal.at(PTZGraph::ld_key, PTZGraph::ld_key)(0)
  };

  const Gaussian<Eigen::Vector2d> pt_scale ={
    Eigen::Vector2d{graph_.estimate.template at<gtsam::Vector2>(PTZGraph::pt_scale_key)},
    joint_marginal.at(PTZGraph::pt_scale_key, PTZGraph::pt_scale_key)
  };

  return {
    cal,
    dt,
    pan_axis,
    tilt_axis,
    line_duration,
    pt_scale,
    pts,
    std::move(updated_landmarks),
    joint_marginal
  };
}

template<typename Cal>
typename PTZEstimator<Cal>::FrameId PTZEstimator<Cal>::createFrameId()
{
  return rot_counter_++;
}

template<typename Cal>
typename PTZEstimator<Cal>::FrameId PTZEstimator<Cal>::insertObservations(
  const NoisyDeltaTimestamp& timestamp,
  const std::map<size_t, Gaussian<Eigen::Vector2d>>& observations
)
{
  if (!prev_frame_id_.has_value())
  {
    prev_frame_id_ = createFrameId();
    graph_.insertOrientationFactor(
      prev_frame_id_.value(),
      {
        timestamp.t.t - timestamp.dt.t,
        Eigen::Vector2d{timestamp.t.sigma_t, timestamp.dt.sigma_t}.norm()
      }
    );
  }

  const auto frame_id = createFrameId();
  const auto rot = graph_.insertOrientationFactor(frame_id, timestamp.t);

  static thread_local const gtsam::Rot3 R_FRD_from_RDF{
    (Eigen::Matrix3d{}
      << Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()
    ).finished()
  };
  const gtsam::Pose3 pose{
    rot*R_FRD_from_RDF,
    Eigen::Vector3d::Zero()
  };

  const auto cal = graph_.template getCalEstimate<Cal>();

  const Camera camera(pose, cal);
  const auto& prev_frame_id = prev_frame_id_.value();
  const auto& [timestep, sigma_timestep] = timestamp.dt;

  for (const auto& [id, uv] : observations)
  {
    const auto landmark_key = PTZGraph::getLandmarkKey(id);

    if (landmark_ids_.find(id) == landmark_ids_.end())
    {
      const auto initial_guess = camera.backprojectPointAtInfinity(uv.x);

      graph_.estimate.insert(
        landmark_key,
        initial_guess
      );
      landmark_ids_.insert(id);
    }

    graph_.template insertProjectionFactor<Cal>(
      id,
      frame_id,
      prev_frame_id,
      timestep,
      sigma_timestep,
      uv
    );
  }

  prev_frame_id_ = frame_id;
  frame_ids_.insert(frame_id);

  return frame_id;
}
}
