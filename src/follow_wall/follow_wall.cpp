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

#include "follow_wall/follow_wall.hpp"

#include <time.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

FollowWallNode::FollowWallNode() : rclcpp_lifecycle::LifecycleNode("follow_wall_lifecycle_node") {
  laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_raw", 10, std::bind(&FollowWallNode::LaserCallback, this, _1));
  pubVelocity_ = this->create_publisher<geometry_msgs::msg::Twist>("/nav_vel", 100);

  srand(time(NULL));
}

int FollowWallNode::angle2pos(float angle, float min, float max, int size) {
  int pos = (angle - min) * size / (max - min);

  if (pos >= size) {
    return pos - 1;
  } else if (pos < 0) {
    return 0;
  }

  return pos;
}

// Calculate the minimun distances values in the center, right and left of
// the robot with range

void FollowWallNode::LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  float min = msg->angle_min;
  float max = msg->angle_max;
  float size = msg->ranges.size();

  float right = angle2pos(-M_PI_2 - M_PI / 36, min, max, size);
  float diag_right = angle2pos(-M_PI_2 + M_PI / 36, min, max, size);

  float left = angle2pos(M_PI_2 + M_PI / 36, min, max, size);
  float diag_left = angle2pos(M_PI_2 - M_PI / 36, min, max, size);

  float center_right = angle2pos(-M_PI / 36, min, max, size);
  float center_left = angle2pos(M_PI / 36, min, max, size);

  float minRight = *std::min_element(std::next(msg->ranges.begin(), right),
                                     std::next(msg->ranges.begin(), diag_right));
  float minCenter = *std::min_element(std::next(msg->ranges.begin(), center_right),
                                      std::next(msg->ranges.begin(), center_left));
  float minLeft = *std::min_element(std::next(msg->ranges.begin(), diag_left),
                                    std::next(msg->ranges.begin(), left));

  std::vector<float> measurements;

  measurements.push_back(minLeft);

  measurements.push_back(minCenter);

  measurements.push_back(minRight);

  laser_regions = measurements;
}

std::vector<float> FollowWallNode::getLaserRegions() {
  return laser_regions;
}

// Check state depend on the distances
void FollowWallNode::CheckState() {
  float side_distance, center_distance;

  center_distance = laser_regions[CENTER];

  if (side_ == LEFT_SIDE) {
    side_distance = laser_regions[LEFT];

  } else {
    side_distance = laser_regions[RIGHT];
  }

  if (side_distance < MIN_DISTANCE || center_distance < MIN_DISTANCE) {
    state_ = TURN_OPPOSITE_SIDE;
  } else if (side_distance > MAX_DISTANCE && center_distance > MIN_DISTANCE) {
    state_ = TURN_SAME_SIDE;

    if (side_distance > 1.5 && center_distance > 1.5) {
      if (counter++ > 100) {
        wall_found = false;
      }
    }
  } else if (side_distance < MAX_DISTANCE && center_distance > MIN_DISTANCE) {
    state_ = GOING_FORWARD;
  } else if (side_distance < MAX_DISTANCE && center_distance < MIN_DISTANCE) {
    if (side_distance < MIN_DISTANCE) {
      state_ = TURN_OPPOSITE_SIDE;
    } else {
      state_ = TURN_SAME_SIDE;
    }
  }

  if (state_ != TURN_SAME_SIDE) {
    counter = 0;
  }
}

void FollowWallNode::FollowTheWall() {
  geometry_msgs::msg::Twist msg;
  float angular_velocity = ANGULAR_VELOCITY;

  if (side_ == RIGHT_SIDE) {
    angular_velocity = -ANGULAR_VELOCITY;
  }

  RCLCPP_WARN(get_logger(), "State: %d", state_);

  switch (state_) {
    case GOING_FORWARD:
      msg.linear.x = LINEAL_VELOCITY;
      msg.angular.z = 0;
      break;

    case TURN_SAME_SIDE:
      msg.linear.x = 0.2;
      msg.angular.z = angular_velocity;
      break;

    case TURN_OPPOSITE_SIDE:
      msg.linear.x = 0;
      msg.angular.z = -angular_velocity;
      break;
  }

  pubVelocity_->publish(msg);
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/* LookFor Wall in this escenary */
void FollowWallNode::LookForWall() {
  geometry_msgs::msg::Twist msg;

  // Case 1: Diagonal la pared
  if (laser_regions[LEFT] < MAX_DISTANCE || laser_regions[CENTER] < MAX_DISTANCE ||
      laser_regions[RIGHT] < MAX_DISTANCE) {
    wall_found = true;
    counter = 0;
  }

  msg.linear.x = LINEAL_VELOCITY;
  msg.angular.z = 0;

  pubVelocity_->publish(msg);
}

CallbackReturnT FollowWallNode::on_configure(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(),
              state.label().c_str());

  // Nos aseguramos que el laser de medidas
  while (laser_regions.size() == 0) {
    continue;
  }

  // Realizamos la configuracion previa para encontrar la pared,
  // comprobando los diferentes casos dependiendo de las medidas de laser

  wall_found = false;
  side_ = RIGHT_SIDE;
  counter = 0;

  if (laser_regions[LEFT] < MAX_DISTANCE) {
    side_ = LEFT_SIDE;
    wall_found = true;
  } else if (laser_regions[RIGHT] < MAX_DISTANCE) {
    side_ = RIGHT_SIDE;
    wall_found = true;
  } else {
    // Si hemos encontrado la pared decidimos aleatoriamente hacia que lado
    // seguir la pared
    int num = rand() % 2;

    if (num == 0) {
      side_ = LEFT_SIDE;
    } else {
      side_ = RIGHT_SIDE;
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowWallNode::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(),
              state.label().c_str());

  pubVelocity_->on_activate();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowWallNode::on_deactivate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
              state.label().c_str());

  pubVelocity_->on_deactivate();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowWallNode::on_cleanup(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(),
              state.label().c_str());
  pubVelocity_.reset();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowWallNode::on_shutdown(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(),
              state.label().c_str());

  pubVelocity_.reset();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowWallNode::on_error(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(),
              state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

void FollowWallNode::do_work() {
  if (laser_regions.size() > 0) {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    if (pubVelocity_->is_activated()) {
      if (!wall_found) {
        LookForWall();
      } else {
        CheckState();
        FollowTheWall();
      }
    }
    RCLCPP_INFO(get_logger(), "Node [%s] active", get_name());
  }
}
