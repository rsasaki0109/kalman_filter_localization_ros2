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

#include <gtest/gtest.h>

#include <kalman_filter_localization/core/state_output.hpp>

namespace
{
using kalman_filter_localization::core::EKFEstimator;
using kalman_filter_localization::core::OutputJacobian;
namespace core = kalman_filter_localization::core;

EKFEstimator::State tiltedState()
{
  EKFEstimator::State state;
  state.orientation = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX());
  state.velocity = Eigen::Vector3d(5.0, -2.0, 1.2);
  return state;
}

EKFEstimator::State perturb(
  EKFEstimator::State state, const int axis, const double delta)
{
  if (axis < 3) {
    state.position(axis) += delta;
  } else if (axis < 6) {
    state.velocity(axis - 3) += delta;
  } else if (axis < 9) {
    state.orientation = state.orientation * Eigen::Quaterniond(
      Eigen::AngleAxisd(delta, Eigen::Vector3d::Unit(axis - 6)));
  } else if (axis < 12) {
    state.gyro_bias(axis - 9) += delta;
  } else {
    state.accel_bias(axis - 12) += delta;
  }
  return state;
}
}  // namespace

TEST(StateOutput, PoseJacobianMatchesFixedReferenceRotationFiniteDifferences)
{
  const auto state = tiltedState();
  OutputJacobian numerical;
  constexpr double epsilon = 1.0e-6;
  for (int axis = 0; axis < EKFEstimator::kErrorStateSize; ++axis) {
    const auto positive = perturb(state, axis, epsilon);
    const auto negative = perturb(state, axis, -epsilon);
    numerical.block<3, 1>(0, axis) =
      (positive.position - negative.position) / (2.0 * epsilon);
    const Eigen::AngleAxisd rotation(positive.orientation * negative.orientation.conjugate());
    numerical.block<3, 1>(3, axis) = rotation.axis() * rotation.angle() / (2.0 * epsilon);
  }
  EXPECT_TRUE(core::poseOutputJacobian(state).isApprox(numerical, 1.0e-8));
}

TEST(StateOutput, PoseCovarianceRotatesAttitudeAndPreservesPositionAttitudeCorrelation)
{
  const auto state = tiltedState();
  EKFEstimator::ErrorStateMatrix factor = EKFEstimator::ErrorStateMatrix::Identity();
  factor(0, 6) = 0.4;
  factor(1, 7) = -0.3;
  factor(8, 8) = 3.0;
  const EKFEstimator::ErrorStateMatrix covariance = factor * factor.transpose();
  const auto output = core::poseOutputCovariance(state, covariance);
  const Eigen::Matrix3d rotation = state.orientation.toRotationMatrix();
  EXPECT_TRUE((output.topRightCorner<3, 3>().isApprox(
      covariance.block<3, 3>(0, 6) * rotation.transpose(), 1.0e-12)));
  EXPECT_GT((output.topRightCorner<3, 3>().norm()), 0.1);
  EXPECT_TRUE((output.bottomRightCorner<3, 3>().isApprox(
      rotation * covariance.block<3, 3>(6, 6) * rotation.transpose(), 1.0e-12)));
  EXPECT_TRUE(output.isApprox(output.transpose(), 1.0e-12));
  EXPECT_EQ(output.llt().info(), Eigen::Success);
}

TEST(StateOutput, BodyTwistJacobianMatchesSixDofFiniteDifferences)
{
  const auto state = tiltedState();
  OutputJacobian numerical;
  constexpr double epsilon = 1.0e-6;
  const Eigen::Vector3d measured_gyro(0.1, -0.2, 0.3);
  for (int axis = 0; axis < EKFEstimator::kErrorStateSize; ++axis) {
    const auto positive = perturb(state, axis, epsilon);
    const auto negative = perturb(state, axis, -epsilon);
    numerical.block<3, 1>(0, axis) =
      (positive.orientation.conjugate() * positive.velocity -
      negative.orientation.conjugate() * negative.velocity) / (2.0 * epsilon);
    numerical.block<3, 1>(3, axis) =
      ((measured_gyro - positive.gyro_bias) - (measured_gyro - negative.gyro_bias)) /
      (2.0 * epsilon);
  }
  EXPECT_TRUE(core::bodyTwistOutputJacobian(state).isApprox(numerical, 1.0e-8));
}

TEST(StateOutput, TwistIncludesAttitudeUncertaintyAndLinearAngularCrossCovariance)
{
  const auto state = tiltedState();
  EKFEstimator::ErrorStateMatrix factor = EKFEstimator::ErrorStateMatrix::Identity();
  factor(3, 9) = 0.5;
  const EKFEstimator::ErrorStateMatrix covariance = factor * factor.transpose();
  const Eigen::Matrix3d noise = 0.02 * Eigen::Matrix3d::Identity();
  const auto output = core::bodyTwistOutputCovariance(state, covariance, noise);
  const Eigen::Matrix3d rotation = state.orientation.toRotationMatrix();
  EXPECT_GT((output.topLeftCorner<3, 3>() -
    rotation.transpose() * covariance.block<3, 3>(3, 3) * rotation).trace(), 1.0);
  EXPECT_GT((output.topRightCorner<3, 3>().norm()), 0.1);
  EXPECT_TRUE((output.bottomRightCorner<3, 3>().isApprox(
      covariance.block<3, 3>(9, 9) + noise, 1.0e-12)));
  EXPECT_EQ(output.llt().info(), Eigen::Success);
}

TEST(StateOutput, MapOdomCompositionPreservesFullSixDofBodyPose)
{
  Eigen::Isometry3d map_body = Eigen::Isometry3d::Identity();
  map_body.linear() = tiltedState().orientation.toRotationMatrix();
  map_body.translation() = Eigen::Vector3d(12.0, -4.0, 2.0);
  Eigen::Isometry3d odom_body = Eigen::Isometry3d::Identity();
  odom_body.linear() = Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  odom_body.translation() = Eigen::Vector3d(3.0, 2.0, 0.5);
  const auto map_odom = core::referenceToOdometry(map_body, odom_body);
  EXPECT_TRUE((map_odom * odom_body).matrix().isApprox(map_body.matrix(), 1.0e-12));
  EXPECT_GT(std::abs(map_odom.translation().z()), 0.1);
}
