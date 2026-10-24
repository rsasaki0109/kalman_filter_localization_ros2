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

#ifndef KALMAN_FILTER_LOCALIZATION__CORE__STATE_OUTPUT_HPP_
#define KALMAN_FILTER_LOCALIZATION__CORE__STATE_OUTPUT_HPP_

#include <kalman_filter_localization/core/ekf_estimator.hpp>

namespace kalman_filter_localization
{
namespace core
{

using OutputCovariance = Eigen::Matrix<double, 6, 6>;
using OutputJacobian = Eigen::Matrix<double, 6, EKFEstimator::kErrorStateSize>;

// ROS Pose covariance uses fixed reference-frame rotation axes. The ESKF uses
// a right-multiplicative BODY tangent: delta_theta_reference = R * delta_theta_body.
inline OutputJacobian poseOutputJacobian(const EKFEstimator::State & state)
{
  OutputJacobian jacobian = OutputJacobian::Zero();
  jacobian.block<3, 3>(0, 0).setIdentity();
  jacobian.block<3, 3>(3, 6) = state.orientation.normalized().toRotationMatrix();
  return jacobian;
}

inline OutputCovariance poseOutputCovariance(
  const EKFEstimator::State & state, const EKFEstimator::ErrorStateMatrix & covariance)
{
  const OutputJacobian jacobian = poseOutputJacobian(state);
  const OutputCovariance result = jacobian * covariance * jacobian.transpose();
  return 0.5 * (result + result.transpose());
}

// Body linear velocity depends on BOTH world velocity and attitude. Body angular
// velocity is the most recent IMU rate minus the estimated gyro bias.
inline OutputJacobian bodyTwistOutputJacobian(const EKFEstimator::State & state)
{
  OutputJacobian jacobian = OutputJacobian::Zero();
  EKFEstimator::ObservationJacobian3 velocity_jacobian;
  (void)EKFEstimator::bodyVelocityObservation(state, &velocity_jacobian);
  jacobian.topRows<3>() = velocity_jacobian;
  jacobian.block<3, 3>(3, 9) = -Eigen::Matrix3d::Identity();
  return jacobian;
}

inline OutputCovariance bodyTwistOutputCovariance(
  const EKFEstimator::State & state, const EKFEstimator::ErrorStateMatrix & covariance,
  const Eigen::Matrix3d & body_gyro_measurement_covariance)
{
  const OutputJacobian jacobian = bodyTwistOutputJacobian(state);
  OutputCovariance result = jacobian * covariance * jacobian.transpose();
  // Standard independent-current-sample approximation. This is not the angular
  // acceleration covariance, nor the gyro process-noise density.
  result.bottomRightCorner<3, 3>() += body_gyro_measurement_covariance;
  return 0.5 * (result + result.transpose());
}

inline Eigen::Isometry3d referenceToOdometry(
  const Eigen::Isometry3d & reference_from_body,
  const Eigen::Isometry3d & odometry_from_body)
{
  return reference_from_body * odometry_from_body.inverse();
}

}  // namespace core
}  // namespace kalman_filter_localization

#endif  // KALMAN_FILTER_LOCALIZATION__CORE__STATE_OUTPUT_HPP_
