// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "mohism.hpp"
namespace mohism {
class LoopClosure {
 struct Impl;std::unique_ptr<Impl> impl_;
public:
 LoopClosure(const Config&,const CameraGeometry&);~LoopClosure();
 // Returns map-frame pose. Local estimator states and marginalization stay continuous.
 Pose update(const Pose& local,const cv::Mat& left,const cv::Mat& right,uint64_t epoch);
 void newEpoch();
 void setRuntimeGeometry(const CameraGeometry&);
 void save(const std::string&)const;
 void load(const std::string&);
 uint64_t correctionVersion()const;
 size_t keyframes()const;uint64_t closures()const;uint64_t relocalizations()const;
};
}
