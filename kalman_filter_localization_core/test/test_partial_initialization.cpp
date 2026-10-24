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

#include <kalman_filter_localization/core/eskf_replay.hpp>
#include <kalman_filter_localization/core/state_initialization.hpp>
#include <kalman_filter_localization/core/state_output.hpp>

namespace
{
namespace core = kalman_filter_localization::core;
using core::EKFEstimator;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
const double kUnknown = std::numeric_limits<double>::quiet_NaN();

Matrix6 correlatedPoseCovariance()
{
  Matrix6 covariance = Matrix6::Identity();
  covariance.diagonal() << 0.4, 0.5, 0.6, 0.01, 0.02, 0.03;
  covariance(0, 3) = covariance(3, 0) = 0.02;
  covariance(1, 5) = covariance(5, 1) = -0.03;
  return covariance;
}
}  // namespace

TEST(SelectedVelocity, ForwardOnlyIgnoresUnknownAxesWithoutConstrainingThemToZero)
{
  EKFEstimator estimator;
  auto before = estimator.getSnapshot();
  before.state.velocity = Eigen::Vector3d(1.0, 2.0, 3.0);
  before.covariance.setIdentity();
  ASSERT_TRUE(estimator.restoreSnapshot(before));
  Eigen::Matrix3d noise = Eigen::Matrix3d::Constant(kUnknown);
  noise(0, 0) = 0.1;
  const auto result = estimator.observationUpdateSelectedBodyVelocity(
    Eigen::Vector3d(1.2, kUnknown, kUnknown), noise, Eigen::Matrix3d::Identity(),
    {{true, false, false}});
  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.dimensions, 1);
  EXPECT_TRUE(std::isfinite(result.innovation_norm));
  EXPECT_TRUE(std::isnan(result.innovation.y()));
  EXPECT_DOUBLE_EQ(estimator.getVelocity().y(), 2.0);
  EXPECT_DOUBLE_EQ(estimator.getVelocity().z(), 3.0);
  EXPECT_DOUBLE_EQ(estimator.getCovariance()(4, 4), 1.0);
  EXPECT_DOUBLE_EQ(estimator.getCovariance()(5, 5), 1.0);
}

TEST(SelectedVelocity, RotatedSingleAxisMatchesProjectedKalmanGain)
{
  EKFEstimator estimator;
  auto snapshot = estimator.getSnapshot();
  snapshot.state.velocity = Eigen::Vector3d(1.0, -0.4, 0.2);
  snapshot.state.orientation = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY());
  snapshot.covariance.setIdentity();
  ASSERT_TRUE(estimator.restoreSnapshot(snapshot));
  const Eigen::Matrix3d source_from_body =
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EKFEstimator::ObservationJacobian3 body_jacobian;
  const Eigen::Vector3d body_velocity = EKFEstimator::bodyVelocityObservation(
    snapshot.state, &body_jacobian);
  const Eigen::Matrix<double, 1, 15> h = (source_from_body * body_jacobian).row(0);
  const double predicted = (source_from_body * body_velocity).x();
  const double variance = (h * snapshot.covariance * h.transpose())(0, 0) + 0.1;
  const EKFEstimator::ErrorStateVector correction =
    snapshot.covariance * h.transpose() / variance * (0.5 - predicted);
  Eigen::Matrix3d noise = Eigen::Matrix3d::Constant(kUnknown);
  noise(0, 0) = 0.1;
  const auto result = estimator.observationUpdateSelectedBodyVelocity(
    Eigen::Vector3d(0.5, kUnknown, kUnknown), noise, source_from_body, {{true, false, false}});
  ASSERT_TRUE(result.accepted);
  EXPECT_NEAR(result.nis, (0.5 - predicted) * (0.5 - predicted) / variance, 1.0e-12);
  EXPECT_TRUE(estimator.getVelocity().isApprox(
      snapshot.state.velocity + correction.segment<3>(3), 1.0e-12));
}

TEST(SelectedVelocity, TwoAxesKeepTheirCorrelatedCovarianceAndRejectBadSelections)
{
  EKFEstimator estimator;
  auto prior = estimator.getSnapshot();
  prior.covariance.setIdentity();
  ASSERT_TRUE(estimator.restoreSnapshot(prior));
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Constant(kUnknown);
  covariance(0, 0) = covariance(2, 2) = 0.1;
  covariance(0, 2) = covariance(2, 0) = 0.03;
  const auto result = estimator.observationUpdateSelectedBodyVelocity(
    Eigen::Vector3d(0.3, kUnknown, 0.4), covariance, Eigen::Matrix3d::Identity(),
    {{true, false, true}});
  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.dimensions, 2);
  Eigen::Matrix2d expected_s;
  expected_s << 1.1, 0.03, 0.03, 1.1;
  const Eigen::Vector2d innovation(0.3, 0.4);
  EXPECT_NEAR(result.nis, innovation.dot(expected_s.ldlt().solve(innovation)), 1.0e-12);
  const auto before = estimator.getSnapshot();
  EXPECT_FALSE(estimator.observationUpdateSelectedBodyVelocity(
      Eigen::Vector3d::Zero(), covariance, Eigen::Matrix3d::Identity(),
      {{false, false, false}}).accepted);
  covariance(0, 0) = -1.0;
  EXPECT_FALSE(estimator.observationUpdateSelectedBodyVelocity(
      Eigen::Vector3d::Zero(), covariance, Eigen::Matrix3d::Identity(),
      {{true, false, false}}).accepted);
  EXPECT_TRUE(estimator.getCovariance().isApprox(before.covariance, 0.0));
}

TEST(SelectedVelocity, DelayedSingleAxisMatchesChronologicalReplay)
{
  EKFEstimator a;
  EKFEstimator b;
  core::EskfReplay chronological(a, 2.0);
  core::EskfReplay delayed(b, 2.0);
  const Eigen::Vector3d gyro(0.02, -0.01, 0.03);
  const Eigen::Vector3d acceleration(0.1, -0.05, 9.80665);
  ASSERT_EQ(chronological.addImu(0, gyro, acceleration), core::EskfReplay::Status::kInitialized);
  ASSERT_EQ(delayed.addImu(0, gyro, acceleration), core::EskfReplay::Status::kInitialized);
  const auto update = [](EKFEstimator & target) {
      Eigen::Matrix3d covariance = Eigen::Matrix3d::Constant(kUnknown);
      covariance(0, 0) = 0.1;
      return target.observationUpdateSelectedBodyVelocity(
        Eigen::Vector3d(0.5, kUnknown, kUnknown), covariance, Eigen::Matrix3d::Identity(),
        {{true, false, false}}).status;
    };
  for (int index = 1; index <= 100; ++index) {
    ASSERT_EQ(chronological.addImu(index * 0.01, gyro, acceleration),
      core::EskfReplay::Status::kApplied);
    ASSERT_EQ(delayed.addImu(index * 0.01, gyro, acceleration), core::EskfReplay::Status::kApplied);
    if (index == 50) {
      ASSERT_EQ(chronological.applyMeasurement(0.5, 0.5, 1U, update),
        core::EskfReplay::Status::kApplied);
    }
  }
  ASSERT_EQ(delayed.applyMeasurement(0.5, 1.0, 1U, update), core::EskfReplay::Status::kApplied);
  EXPECT_TRUE(a.getPosition().isApprox(b.getPosition(), 1.0e-10));
  EXPECT_TRUE(a.getVelocity().isApprox(b.getVelocity(), 1.0e-10));
  EXPECT_TRUE(a.getCovariance().isApprox(b.getCovariance(), 1.0e-10));
}

TEST(PoseInitialization, FullCovarianceRoundTripsUnderTiltAndClearsPredictionHistory)
{
  EKFEstimator estimator;
  ASSERT_EQ(estimator.predictionUpdateDt(
      0.01, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.80665)),
    EKFEstimator::PredictionUpdateStatus::kUpdated);
  ASSERT_TRUE(estimator.hasLastDiscreteModel());
  EKFEstimator::Pose pose;
  pose.position = Eigen::Vector3d(4.0, -2.0, 1.0);
  pose.orientation = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitX());
  const Matrix6 covariance = correlatedPoseCovariance();
  ASSERT_TRUE(core::initializePoseWithCovariance(
      estimator, pose, covariance, Eigen::Vector3d::Ones(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(0.01)));
  const auto snapshot = estimator.getSnapshot();
  EXPECT_TRUE(core::poseOutputCovariance(snapshot.state, snapshot.covariance).isApprox(
      covariance, 1.0e-12));
  EXPECT_TRUE(snapshot.state.velocity.isZero(0.0));
  EXPECT_TRUE(snapshot.state.gyro_bias.isZero(0.0));
  EXPECT_TRUE(snapshot.state.accel_bias.isZero(0.0));
  EXPECT_FALSE(snapshot.has_previous_time_imu);
  EXPECT_FALSE(snapshot.has_previous_imu_measurement);
  EXPECT_FALSE(estimator.hasLastDiscreteModel());
}

TEST(PoseInitialization, InvalidRequestsPreserveStateCovarianceAndCachedModel)
{
  EKFEstimator estimator;
  ASSERT_EQ(estimator.predictionUpdateDt(
      0.01, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.80665)),
    EKFEstimator::PredictionUpdateStatus::kUpdated);
  const auto before = estimator.getSnapshot();
  EKFEstimator::Pose pose;
  for (int kind = 0; kind < 4; ++kind) {
    Matrix6 covariance = correlatedPoseCovariance();
    pose.orientation = Eigen::Quaterniond::Identity();
    if (kind == 0) {covariance(0, 0) = -1;}
    if (kind == 1) {covariance(0, 3) = 2;}
    if (kind == 2) {pose.orientation = Eigen::Quaterniond(0, 0, 0, 0);}
    if (kind == 3) {pose.orientation = Eigen::Quaterniond(1e308, 1e308, 0, 0);}
    EXPECT_FALSE(core::initializePoseWithCovariance(
        estimator, pose, covariance, Eigen::Vector3d::Ones(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
    EXPECT_TRUE(estimator.getPosition().isApprox(before.state.position, 0.0));
    EXPECT_TRUE(estimator.getCovariance().isApprox(before.covariance, 0.0));
    EXPECT_TRUE(estimator.hasLastDiscreteModel());
  }
}

TEST(PoseInitialization, ExplicitConfiguredZeroVarianceRemainsSupported)
{
  EKFEstimator estimator;
  EKFEstimator::Pose pose;
  EXPECT_TRUE(core::initializePoseWithCovariance(
      estimator, pose, Matrix6::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false));
  EXPECT_FALSE(core::initializePoseWithCovariance(
      estimator, pose, Matrix6::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
}
