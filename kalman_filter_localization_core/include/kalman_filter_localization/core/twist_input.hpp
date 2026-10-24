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

#ifndef KALMAN_FILTER_LOCALIZATION__CORE__TWIST_INPUT_HPP_
#define KALMAN_FILTER_LOCALIZATION__CORE__TWIST_INPUT_HPP_

#include <kalman_filter_localization/core/ekf_estimator.hpp>

namespace kalman_filter_localization
{
namespace core
{

// Source-frame twist is the velocity of the SOURCE ORIGIN. The rigid-body
// transport to the body origin is v_body = R*v_source + r cross (R*w_source).
// This uses the twist's own angular sample, not the IMU propagation sample.
inline Eigen::Matrix<double, 3, 6> twistLinearTransportJacobian(
  const Eigen::Isometry3d & body_from_source)
{
  Eigen::Matrix<double, 3, 6> result;
  result.leftCols<3>() = body_from_source.linear();
  result.rightCols<3>() = EKFEstimator::skewSymmetric(body_from_source.translation()) *
    body_from_source.linear();
  return result;
}

}  // namespace core
}  // namespace kalman_filter_localization

#endif  // KALMAN_FILTER_LOCALIZATION__CORE__TWIST_INPUT_HPP_
