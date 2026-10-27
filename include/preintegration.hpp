// SPDX-License-Identifier: GPL-3.0-only
// Adapted from VINS-Fusion IntegrationBase; see third_party/VINS-Fusion/NOTICE.md.
#pragma once
#include "mohism.hpp"

namespace mohism {
// Bias-linearized deltas in the first IMU frame. Error order: p, theta, v, ba, bg.
class Preintegration {
 ImuSample first_,last_;
 V3 ba_,bg_;
 double acc_noise_,gyro_noise_;
 struct Step {double dt;ImuSample sample;};
 std::vector<Step> history_;
 void integrate(double dt,const ImuSample& sample);
public:
 V3 dp{},dv{};M3 dR=M3::eye();double duration=0;
 cv::Mat jacobian,covariance;
 Preintegration(const ImuSample& first,const V3& ba={},const V3& bg={},
                double acc_noise=.1,double gyro_noise=.01);
 void push(const ImuSample& sample);
 void repropagate(const V3& ba,const V3& bg);
};
bool finite(const V3& v);
bool validImu(const ImuSample& sample);
bool rotation(const M3& r);
}
