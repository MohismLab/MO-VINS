// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "mohism.hpp"
#include <optional>
namespace mohism {
struct InitialState {Pose pose;V3 ba{},bg{};ImuSample endpoint;};
// Stereo fixes metric scale; align metric visual poses with buffered IMU to solve
// gyro bias, velocities, gravity and accelerometer bias before starting the window.
class StereoInitializer {
 std::deque<Pose> frames_;
public:
 std::optional<InitialState> add(const Pose& visual,const std::deque<ImuSample>& imu);
};
}
