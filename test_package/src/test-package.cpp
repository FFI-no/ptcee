#include "ptcee/cal_fk.h"
#include "ptcee/ptz_estimator.h"

#include <iostream>

using Cal = ptc::CalFK;
using Camera = gtsam::PinholeCamera<Cal>;
using PTZEstimator = ptc::PTZEstimator<Cal>;

int main(const int, const char**)
{
  PTZEstimator estimator{
    {Cal{1e3, 1e3, 1920, 1080}, Eigen::Matrix2d::Identity()},
    {0., 1e-6},
    {0., 1e-6},
    Eigen::Vector2d::Ones(),
    1e-6*Eigen::Array2d::Ones(),
    true
  };

  estimator.estimate();

  const auto& estimate = estimator.getRawEstimate();
  const auto& cal = estimate.at<Cal>(ptc::PTZGraph::cal_key);

  std::cout
    << "estimated"
    << " f: " << cal.f
    << " k: " << cal.k
    << " w: " << cal.w
    << " h: " << cal.h
    << std::endl;

  return 0;
}
