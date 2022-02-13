// Copyright 2022 K-Bots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FOLLOW_WALL__FOLLOW_WALL_HPP_

#define FOLLOW_WALL__FOLLOW_WALL_HPP_

#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class FollowWallNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  FollowWallNode();

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  CallbackReturnT on_configure(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State& state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State& state);
  int angle2pos(float angle, float min, float max, int size);
  void CheckState();
  void FollowTheWall();
  void do_work();
  void LookForWall();
  std::vector<float> getLaserRegions();

  std::vector<float> laser_regions;

 private:
  std::vector<float> laser_regions_prev;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pubVelocity_;

  const float LINEAL_VELOCITY = 0.6;
  const float ANGULAR_VELOCITY = 1;
  const float MAX_DISTANCE = 0.5;
  const float MIN_DISTANCE = 0.4;

  bool wall_found = false;

  int counter = 0;
  enum robot_side { LEFT_SIDE = 1, RIGHT_SIDE };

  enum movement {
    GOING_FORWARD = 1,
    TURN_SAME_SIDE,  // wall side
    TURN_OPPOSITE_SIDE,
    WALL_AFRONT
  };

  enum laser_side { LEFT = 0, CENTER, RIGHT };

  int state_ = GOING_FORWARD;
  int side_ = 0;
};

#endif  // FOLLOW_WALL__FOLLOW_WALL_HPP_
