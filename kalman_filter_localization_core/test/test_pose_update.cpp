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

#include <limits>

#include <kalman_filter_localization/core/ekf_estimator.hpp>
#include <kalman_filter_localization/core/eskf_replay.hpp>

namespace
{
using kalman_filter_localization::core::EKFEstimator;
using kalman_filter_localization::core::EskfReplay;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

Matrix6 correlatedCovariance()
{
  Matrix6 result = Matrix6::Identity();
  result(0, 3) = result(3, 0) = 0.6;
  result(1, 4) = result(4, 1) = -0.4;
  result(2, 5) = result(5, 2) = 0.3;
  return result;
}

void expectUnchanged(const EKFEstimator & estimator, const EKFEstimator::Snapshot & before)
{
  const auto after = estimator.getSnapshot();
  EXPECT_TRUE(after.state.position.isApprox(before.state.position, 0.0));
  EXPECT_TRUE(after.state.orientation.coeffs().isApprox(before.state.orientation.coeffs(), 0.0));
  EXPECT_TRUE(after.covariance.isApprox(before.covariance, 0.0));
}
}  // namespace

TEST(PoseUpdate, JointCorrelatedUpdateMatchesSixDimensionalKalmanGain)
{
  EKFEstimator estimator;
  auto prior = estimator.getSnapshot();
  prior.covariance.setIdentity();
  ASSERT_TRUE(estimator.restoreSnapshot(prior));
  const auto before = estimator.getSnapshot();
  const Eigen::Vector3d position(0.2, -0.1, 0.3);
  const Eigen::Vector3d angle(0.04, -0.03, 0.02);
  const Eigen::Quaterniond orientation(Eigen::AngleAxisd(angle.norm(), angle.normalized()));
  Eigen::Matrix<double, 6, 15> jacobian = Eigen::Matrix<double, 6, 15>::Zero();
  jacobian.block<3, 3>(0, 0).setIdentity();
  jacobian.block<3, 3>(3, 6).setIdentity();
  Eigen::Matrix<double, 6, 1> innovation;
  innovation << position, angle;
  const Matrix6 noise = correlatedCovariance();
  const Matrix6 innovation_covariance = jacobian * before.covariance * jacobian.transpose() + noise;
  const Eigen::Matrix<double, 15, 6> gain = innovation_covariance.ldlt().solve(
    (before.covariance * jacobian.transpose()).transpose()).transpose();
  const EKFEstimator::ErrorStateVector expected = gain * innovation;
  const auto diagnostics = estimator.observationUpdatePoseWithCovariance(
    position, orientation, noise);
  ASSERT_TRUE(diagnostics.accepted);
  EXPECT_TRUE(diagnostics.innovation.isApprox(innovation, 1.0e-12));
  EXPECT_NEAR(diagnostics.nis, innovation.dot(innovation_covariance.ldlt().solve(innovation)),
    1e-12);
  EXPECT_TRUE(estimator.getState().position.isApprox(expected.head<3>(), 1.0e-12));
  const Eigen::Vector3d correction = expected.segment<3>(6);
  const Eigen::Quaterniond expected_orientation(
    Eigen::AngleAxisd(correction.norm(), correction.normalized()));
  EXPECT_LT(estimator.getState().orientation.angularDistance(expected_orientation), 1.0e-12);
  EXPECT_TRUE(estimator.checkNumericalInvariants());

  EKFEstimator diagonal;
  ASSERT_TRUE(diagonal.restoreSnapshot(before));
  ASSERT_TRUE(diagonal.observationUpdatePoseWithCovariance(
      position, orientation, Matrix6(noise.diagonal().asDiagonal())).accepted);
  EXPECT_GT((estimator.getPosition() - diagonal.getPosition()).norm(), 1.0e-3);
}

TEST(PoseUpdate, WorldCovarianceIsMappedIntoTiltedBodyTangentWithAllCrossTerms)
{
  EKFEstimator estimator;
  auto state = estimator.getState();
  state.orientation = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY());
  estimator.setState(state);
  const auto before = estimator.getSnapshot();
  Matrix6 mapping = Matrix6::Identity();
  mapping.bottomRightCorner<3, 3>() = state.orientation.toRotationMatrix().transpose();
  const Matrix6 expected_noise = mapping * correlatedCovariance() * mapping.transpose();
  Eigen::Matrix<double, 6, 15> jacobian = Eigen::Matrix<double, 6, 15>::Zero();
  jacobian.block<3, 3>(0, 0).setIdentity();
  jacobian.block<3, 3>(3, 6).setIdentity();
  const auto diagnostics = estimator.observationUpdatePoseWithCovariance(
    state.position, state.orientation, correlatedCovariance());
  ASSERT_TRUE(diagnostics.accepted);
  EXPECT_TRUE(diagnostics.innovation_covariance.isApprox(
      jacobian * before.covariance * jacobian.transpose() + expected_noise, 1.0e-12));
}

TEST(PoseUpdate, AntipodalQuaternionsProduceIdenticalUpdates)
{
  EKFEstimator positive;
  EKFEstimator negative;
  const Eigen::Quaterniond orientation(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond antipodal = orientation;
  antipodal.coeffs() *= -1.0;
  ASSERT_TRUE(positive.observationUpdatePoseWithCovariance(
      Eigen::Vector3d(0.1, 0.2, 0.3), orientation, correlatedCovariance()).accepted);
  ASSERT_TRUE(negative.observationUpdatePoseWithCovariance(
      Eigen::Vector3d(0.1, 0.2, 0.3), antipodal, correlatedCovariance()).accepted);
  EXPECT_TRUE(positive.getCovariance().isApprox(negative.getCovariance(), 1.0e-12));
  EXPECT_TRUE(positive.getPosition().isApprox(negative.getPosition(), 1.0e-12));
  EXPECT_LT(positive.getOrientation().angularDistance(negative.getOrientation()), 1.0e-12);
}

TEST(PoseUpdate, InvalidMeasurementCovarianceAndNisGateNeverMutateState)
{
  EKFEstimator estimator;
  const auto before = estimator.getSnapshot();
  for (const int kind : {0, 1, 2, 3}) {
    Matrix6 covariance = Matrix6::Identity();
    if (kind == 0) {covariance(0, 0) = -1.0;}
    if (kind == 1) {covariance(0, 1) = 0.3;}
    if (kind == 2) {covariance(5, 5) = 0.0;}
    if (kind == 3) {covariance(2, 2) = std::numeric_limits<double>::quiet_NaN();}
    const auto result = estimator.observationUpdatePoseWithCovariance(
      Eigen::Vector3d::Ones(), Eigen::Quaterniond::Identity(), covariance);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.reason, EKFEstimator::ObservationRejectReason::kInvalidCovariance);
    expectUnchanged(estimator, before);
  }
  const auto zero_quaternion = estimator.observationUpdatePoseWithCovariance(
    Eigen::Vector3d::Zero(), Eigen::Quaterniond(0, 0, 0, 0), Matrix6::Identity());
  EXPECT_FALSE(zero_quaternion.accepted);
  expectUnchanged(estimator, before);
  const auto overflowing_quaternion = estimator.observationUpdatePoseWithCovariance(
    Eigen::Vector3d::Zero(), Eigen::Quaterniond(1e308, 1e308, 0, 0), Matrix6::Identity());
  EXPECT_FALSE(overflowing_quaternion.accepted);
  expectUnchanged(estimator, before);
  const auto gated = estimator.observationUpdatePoseWithCovariance(
    Eigen::Vector3d::Constant(100.0), Eigen::Quaterniond::Identity(), Matrix6::Identity(), 1.0);
  EXPECT_FALSE(gated.accepted);
  EXPECT_EQ(gated.reason, EKFEstimator::ObservationRejectReason::kNisGate);
  expectUnchanged(estimator, before);
}

TEST(PoseUpdate, DelayedPoseReplayMatchesChronologicalFullStateAndCovariance)
{
  EKFEstimator chronological;
  EKFEstimator delayed;
  for (auto * estimator : {&chronological, &delayed}) {
    estimator->setPropagationModel(EKFEstimator::PropagationModel::kFast);
  }
  EskfReplay reference(chronological, 3.0);
  EskfReplay replay(delayed, 3.0);
  const Eigen::Vector3d gyro(0.02, -0.01, 0.03);
  const Eigen::Vector3d acceleration(0.1, -0.05, 9.80665);
  ASSERT_EQ(reference.addImu(0.0, gyro, acceleration), EskfReplay::Status::kInitialized);
  ASSERT_EQ(replay.addImu(0.0, gyro, acceleration), EskfReplay::Status::kInitialized);
  const auto update = [](EKFEstimator & estimator) {
      return estimator.observationUpdatePoseWithCovariance(
        Eigen::Vector3d(0.1, -0.2, 0.3),
        Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX())),
        correlatedCovariance()).status;
    };
  for (int index = 1; index <= 100; ++index) {
    ASSERT_EQ(reference.addImu(index * 0.01, gyro, acceleration), EskfReplay::Status::kApplied);
    ASSERT_EQ(replay.addImu(index * 0.01, gyro, acceleration), EskfReplay::Status::kApplied);
    if (index == 50) {
      ASSERT_EQ(reference.applyMeasurement(0.5, 0.5, 1U, update), EskfReplay::Status::kApplied);
    }
  }
  ASSERT_EQ(replay.applyMeasurement(0.5, 1.0, 1U, update), EskfReplay::Status::kApplied);
  EXPECT_TRUE(chronological.getPosition().isApprox(delayed.getPosition(), 1.0e-10));
  EXPECT_TRUE(chronological.getVelocity().isApprox(delayed.getVelocity(), 1.0e-10));
  EXPECT_LT(chronological.getOrientation().angularDistance(delayed.getOrientation()), 1.0e-10);
  EXPECT_TRUE(chronological.getCovariance().isApprox(delayed.getCovariance(), 1.0e-10));
}

TEST(PoseUpdate, FuturePoseQueuesUntilImuAndMatchesChronologicalUpdate)
{
  EKFEstimator reference_estimator;
  EKFEstimator queued_estimator;
  EskfReplay reference(reference_estimator, 2.0, 0.2);
  EskfReplay queued(queued_estimator, 2.0, 0.2);
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acceleration(0.0, 0.0, 9.80665);
  ASSERT_EQ(reference.addImu(0.0, gyro, acceleration), EskfReplay::Status::kInitialized);
  ASSERT_EQ(queued.addImu(0.0, gyro, acceleration), EskfReplay::Status::kInitialized);
  const auto update = [](EKFEstimator & estimator) {
      return estimator.observationUpdatePoseWithCovariance(
        Eigen::Vector3d(0.1, -0.2, 0.3),
        Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY())),
        correlatedCovariance()).status;
    };
  ASSERT_EQ(queued.applyMeasurement(0.1, 0.0, 1U, update), EskfReplay::Status::kQueuedFuture);
  for (int index = 1; index <= 20; ++index) {
    ASSERT_EQ(reference.addImu(index * 0.01, gyro, acceleration), EskfReplay::Status::kApplied);
    ASSERT_EQ(queued.addImu(index * 0.01, gyro, acceleration), EskfReplay::Status::kApplied);
    if (index == 10) {
      ASSERT_EQ(reference.applyMeasurement(0.1, 0.1, 1U, update), EskfReplay::Status::kApplied);
    }
  }
  EXPECT_EQ(queued.counters().future_queued, 1U);
  EXPECT_TRUE(reference_estimator.getPosition().isApprox(queued_estimator.getPosition(), 1.0e-10));
  EXPECT_TRUE(reference_estimator.getCovariance().isApprox(
      queued_estimator.getCovariance(), 1.0e-10));
}
