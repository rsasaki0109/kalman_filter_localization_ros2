// Copyright (c) 2026, Ryohei Sasaki
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef KALMAN_FILTER_LOCALIZATION__CORE__STATE_INITIALIZATION_HPP_
#define KALMAN_FILTER_LOCALIZATION__CORE__STATE_INITIALIZATION_HPP_

#include <algorithm>
#include <cmath>

#include <kalman_filter_localization/core/ekf_estimator.hpp>

namespace kalman_filter_localization
{
namespace core
{

// Atomic initialization. Input pose covariance uses fixed world rotation axes;
// the error state uses a right/body tangent. Process-model configuration stays.
inline bool initializePoseWithCovariance(
  EKFEstimator & estimator, const EKFEstimator::Pose & pose,
  const Eigen::Matrix<double, 6, 6> & world_covariance,
  const Eigen::Vector3d & velocity_variance,
  const Eigen::Vector3d & gyro_bias_variance,
  const Eigen::Vector3d & accel_bias_variance,
  const bool require_positive_pose_covariance = true)
{
  const double norm = pose.orientation.norm();
  if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
    !std::isfinite(norm) || !(norm > 1.0e-12) || !world_covariance.allFinite() ||
    !world_covariance.isApprox(world_covariance.transpose(), 1.0e-12))
  {
    return false;
  }
  for (const auto & variance : {velocity_variance, gyro_bias_variance, accel_bias_variance}) {
    if (!variance.allFinite() || (variance.array() < 0.0).any()) {return false;}
  }
  if (require_positive_pose_covariance) {
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> decomposition(world_covariance);
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive() ||
      !(decomposition.vectorD().minCoeff() > 1.0e-12 * decomposition.vectorD().maxCoeff()))
    {
      return false;
    }
  } else {
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(world_covariance);
    if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() <
      -1.0e-12 * std::max(1.0, solver.eigenvalues().cwiseAbs().maxCoeff()))
    {
      return false;
    }
  }
  EKFEstimator::Snapshot initial;
  initial.state.position = pose.position;
  initial.state.orientation = pose.orientation.normalized();
  initial.covariance.setZero();
  Eigen::Matrix<double, 6, 6> mapping = Eigen::Matrix<double, 6, 6>::Identity();
  mapping.bottomRightCorner<3, 3>() = initial.state.orientation.toRotationMatrix().transpose();
  const Eigen::Matrix<double, 6, 6> tangent_covariance =
    mapping * world_covariance * mapping.transpose();
  const int indices[] = {0, 1, 2, 6, 7, 8};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      initial.covariance(indices[row], indices[col]) = tangent_covariance(row, col);
    }
  }
  initial.covariance.block<3, 3>(3, 3).diagonal() = velocity_variance;
  initial.covariance.block<3, 3>(9, 9).diagonal() = gyro_bias_variance;
  initial.covariance.block<3, 3>(12, 12).diagonal() = accel_bias_variance;
  if (!estimator.restoreSnapshot(initial)) {return false;}
  estimator.resetImuTimeBase();
  return true;
}

}  // namespace core
}  // namespace kalman_filter_localization

#endif  // KALMAN_FILTER_LOCALIZATION__CORE__STATE_INITIALIZATION_HPP_
