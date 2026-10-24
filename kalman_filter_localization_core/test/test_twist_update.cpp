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

#include <kalman_filter_localization/core/eskf_replay.hpp>
#include <kalman_filter_localization/core/twist_input.hpp>

namespace
{
using kalman_filter_localization::core::EKFEstimator;
using kalman_filter_localization::core::EskfReplay;
namespace core = kalman_filter_localization::core;
}

TEST(TwistUpdate, CorrelatedBodyVelocityMatchesKalmanGainUnderTilt)
{
  EKFEstimator estimator;
  auto snapshot = estimator.getSnapshot();
  snapshot.covariance.setIdentity();
  snapshot.state.orientation = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX());
  snapshot.state.velocity = Eigen::Vector3d(2.0, -0.5, 0.8);
  ASSERT_TRUE(estimator.restoreSnapshot(snapshot));
  Eigen::Matrix3d noise = Eigen::Matrix3d::Identity();
  noise(0, 2) = noise(2, 0) = 0.4;
  EKFEstimator::ObservationJacobian3 jacobian;
  const Eigen::Vector3d predicted = EKFEstimator::bodyVelocityObservation(snapshot.state,
    &jacobian);
  const Eigen::Vector3d measured(1.8, -0.1, 0.5);
  const Eigen::Matrix3d innovation_covariance =
    jacobian * snapshot.covariance * jacobian.transpose() + noise;
  const Eigen::Matrix<double, 15, 3> gain = innovation_covariance.ldlt().solve(
    (snapshot.covariance * jacobian.transpose()).transpose()).transpose();
  const EKFEstimator::ErrorStateVector expected = gain * (measured - predicted);
  const auto result = estimator.observationUpdateBodyVelocityWithCovariance(measured, noise);
  ASSERT_TRUE(result.accepted);
  EXPECT_TRUE(result.innovation_covariance.isApprox(innovation_covariance, 1.0e-12));
  EXPECT_TRUE(estimator.getVelocity().isApprox(
      snapshot.state.velocity + expected.segment<3>(3), 1.0e-12));
  EXPECT_GT(estimator.getOrientation().angularDistance(snapshot.state.orientation), 0.001);
  EXPECT_TRUE(estimator.checkNumericalInvariants());
}

TEST(TwistUpdate, RigidSensorTransportCancelsLeverArmRotationVelocity)
{
  Eigen::Isometry3d body_from_sensor = Eigen::Isometry3d::Identity();
  body_from_sensor.translation() = Eigen::Vector3d(1.0, -0.3, 0.2);
  body_from_sensor.linear() =
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d body_velocity(1.2, 0.3, -0.2);
  const Eigen::Vector3d angular_velocity(0.1, -0.2, 0.4);
  Eigen::Matrix<double, 6, 1> measured;
  measured.head<3>() = body_from_sensor.linear().transpose() *
    (body_velocity + angular_velocity.cross(body_from_sensor.translation()));
  measured.tail<3>() = body_from_sensor.linear().transpose() * angular_velocity;
  const auto jacobian = core::twistLinearTransportJacobian(body_from_sensor);
  EXPECT_TRUE((jacobian * measured).isApprox(body_velocity, 1.0e-12));
}

TEST(TwistUpdate, TransportCovariancePreservesLinearAngularCorrelation)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.8, -0.2, 0.3);
  const auto jacobian = core::twistLinearTransportJacobian(transform);
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
  covariance(0, 5) = covariance(5, 0) = 0.4;
  const Eigen::Matrix3d result = jacobian * covariance * jacobian.transpose();
  Eigen::Matrix3d independent = jacobian.leftCols<3>() * jacobian.leftCols<3>().transpose() +
    jacobian.rightCols<3>() * jacobian.rightCols<3>().transpose();
  EXPECT_GT((result - independent).norm(), 0.1);
  EXPECT_EQ(result.llt().info(), Eigen::Success);
}

TEST(TwistUpdate, InvalidCovarianceAndNisGateDoNotMutateEstimator)
{
  EKFEstimator estimator;
  const auto before = estimator.getSnapshot();
  Eigen::Matrix3d invalid = Eigen::Matrix3d::Identity();
  invalid(0, 0) = -1.0;
  EXPECT_FALSE(estimator.observationUpdateBodyVelocityWithCovariance(
      Eigen::Vector3d::Ones(), invalid).accepted);
  const auto gated = estimator.observationUpdateBodyVelocityWithCovariance(
    Eigen::Vector3d::Constant(100), Eigen::Matrix3d::Identity(), 1.0);
  EXPECT_EQ(gated.reason, EKFEstimator::ObservationRejectReason::kNisGate);
  EXPECT_FALSE(gated.accepted);
  EXPECT_TRUE(estimator.getCovariance().isApprox(before.covariance, 0.0));
  EXPECT_TRUE(estimator.getVelocity().isApprox(before.state.velocity, 0.0));
}

TEST(TwistUpdate, DelayedVelocityMatchesChronologicalStateAndCovariance)
{
  EKFEstimator a;
  EKFEstimator b;
  EskfReplay chronological(a, 2.0);
  EskfReplay delayed(b, 2.0);
  const Eigen::Vector3d gyro(0.02, -0.01, 0.05);
  const Eigen::Vector3d acceleration(0.1, -0.05, 9.80665);
  ASSERT_EQ(chronological.addImu(0, gyro, acceleration), EskfReplay::Status::kInitialized);
  ASSERT_EQ(delayed.addImu(0, gyro, acceleration), EskfReplay::Status::kInitialized);
  const auto update = [](EKFEstimator & estimator) {
      return estimator.observationUpdateBodyVelocityWithCovariance(
        Eigen::Vector3d(1.2, -0.3, 0.4), 0.1 * Eigen::Matrix3d::Identity()).status;
    };
  for (int index = 1; index <= 100; ++index) {
    ASSERT_EQ(chronological.addImu(index * 0.01, gyro, acceleration), EskfReplay::Status::kApplied);
    ASSERT_EQ(delayed.addImu(index * 0.01, gyro, acceleration), EskfReplay::Status::kApplied);
    if (index == 50) {
      ASSERT_EQ(chronological.applyMeasurement(0.5, 0.5, 1, update), EskfReplay::Status::kApplied);
    }
  }
  ASSERT_EQ(delayed.applyMeasurement(0.5, 1.0, 1, update), EskfReplay::Status::kApplied);
  EXPECT_TRUE(a.getPosition().isApprox(b.getPosition(), 1.0e-10));
  EXPECT_TRUE(a.getVelocity().isApprox(b.getVelocity(), 1.0e-10));
  EXPECT_LT(a.getOrientation().angularDistance(b.getOrientation()), 1.0e-10);
  EXPECT_TRUE(a.getCovariance().isApprox(b.getCovariance(), 1.0e-10));
}
