// SPDX-License-Identifier: GPL-3.0-only
// State ordering retained from the pinned VINS-Fusion factors; no ROS globals.
#pragma once
enum StateOrder { O_P=0,O_R=3,O_V=6,O_BA=9,O_BG=12 };
