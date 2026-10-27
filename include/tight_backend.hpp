// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "mohism.hpp"
namespace mohism {
class TightBackend {
 struct Impl;std::unique_ptr<Impl> impl_;
public:
 TightBackend(const Config&,const CameraGeometry&,const Pose&,const V3& ba,const V3& bg,const ImuSample& endpoint);
 ~TightBackend();
 Pose process(double timestamp,const std::vector<Observation>& observations,const std::vector<ImuSample>& imu);
 void setNoise(const std::array<double,4>&);
 double timeOffset()const;
 CameraGeometry geometry()const;
 Estimator::Diagnostics diagnostics()const;
};
}
