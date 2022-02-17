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
#include "follow_wall/follow_wall.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

FollowWallNode::FollowWallNode()
: rclcpp_lifecycle::LifecycleNode("follow_wall_lifecycle_node")
{
  laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan_filtered", 10, std::bind(&FollowWallNode::LaserCallback, this, _1));
}

int FollowWallNode::angle2pos(float angle, float min, float max, int size)
{
  int pos = (angle - min) * size / (max - min);

  if (pos >= size) {
    return size - 1;
  } else if (pos < 0) {
    return 0;
  }

  return pos;
}

// If a is less than b returns true
bool FollowWallNode::comparator(float a, float b)
{
  // If a is inf or nan return false
  if (std::isnan(a) || std::isinf(a)) {
    return false;
  }

  // If a is a number and b is nan or inf, return true
  if (std::isinf(b) || std::isnan(b)) {
    return true;
  }

  // If both are numbers return the value of the comparison
  return a < b;
}
// Calculate the minimun distances values in the center, right and left of
// the robot with range

void FollowWallNode::LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  float min = msg->angle_min;
  float max = msg->angle_max;
  float size = msg->ranges.size();

  float right = angle2pos(-M_PI_2 - M_PI / 18.0, min, max, size);
  float diag_right = angle2pos(-M_PI_2 + M_PI / 18.0, min, max, size);

  float left = angle2pos(M_PI_2 + M_PI / 18.0, min, max, size);
  float diag_left = angle2pos(M_PI_2 - M_PI / 18.0, min, max, size);

  float center_right = angle2pos(M_PI - M_PI / 10.0, min, max, size);
  float center_left = angle2pos(M_PI + M_PI / 10.0, min, max, size);

  float minRight = *std::min_element(
    std::next(msg->ranges.begin(), right),
    std::next(msg->ranges.begin(), diag_right), FollowWallNode::comparator);
  float minCenter = *std::min_element(
    std::next(msg->ranges.begin(), center_right),
    std::next(msg->ranges.begin(), center_left), FollowWallNode::comparator);
  float minLeft = *std::min_element(
    std::next(msg->ranges.begin(), diag_left),
    std::next(msg->ranges.begin(), left), FollowWallNode::comparator);

  if (std::isnan(minLeft)) {
    minLeft = NAN_DISTANCE;
  }

 if (std::isnan(minCenter)) {
    minCenter = NAN_DISTANCE;
  }

  if (std::isnan(minRight)) {
    minRight = NAN_DISTANCE;
  }

  if (std::isinf(minLeft)) {
    minLeft = INF_DISTANCE;
  }

  if (std::isinf(minCenter)) {
    minCenter = INF_DISTANCE;
  }

  if (std::isinf(minRight)) {
    minRight = INF_DISTANCE;
  }


  std::vector<float> measurements;


  measurements.push_back(minLeft);
  measurements.push_back(minCenter);
  measurements.push_back(minRight);

  RCLCPP_WARN(get_logger(), "%f %f %f\n",minLeft,minCenter,minRight);

  laser_regions = measurements;
}

// Check state depend on the distances
void FollowWallNode::CheckState()
{
  float side_distance, center_distance;

  center_distance = laser_regions[CENTER];

  if (side_ == LEFT_SIDE) {
    side_distance = laser_regions[LEFT];
  } else {
    side_distance = laser_regions[RIGHT];
  }

  if (center_distance < MIN_DISTANCE) {
    state_ = WALL_AHEAD;
  } else {
    // Too close to wall, move away
    if (side_distance < MIN_DISTANCE) {
      state_ = TURN_OPPOSITE_SIDE;
    } else if (side_distance < MAX_DISTANCE) {
      state_ = GOING_FORWARD;
    } else {
      state_ = TURN_SAME_SIDE;

      // Kidnapped situation
      if (side_distance > RESTART_VALUE && center_distance > RESTART_VALUE) {
        if (counter_++ > MAX_ITERATIONS) {
          wall_found = false;
        }
      }
    }
  }

  if (state_ != TURN_SAME_SIDE) {
    counter_ = 0;
  }
}

void FollowWallNode::SelectSide(){
  state_configure = true;
  wall_found = false;
  counter_ = 0;

  if (laser_regions[LEFT] < MAX_DISTANCE) {
    side_ = LEFT_SIDE;
    wall_found = true;
  } else if (laser_regions[RIGHT] < MAX_DISTANCE) {
    side_ = RIGHT_SIDE;
    wall_found = true;
  } else {
    unsigned int seed = time(NULL);
    int num = rand_r(&seed) % 2;

    if (num == 0) {
      side_ = LEFT_SIDE;
    } else {
      side_ = RIGHT_SIDE;
    }
  }

  if (side_ == LEFT_SIDE) {
    RCLCPP_INFO(get_logger(), "Side: Left");
  } else {
    RCLCPP_INFO(get_logger(), "Side: Right");
  }

  

}

// Finite state machine
void FollowWallNode::FollowTheWall()
{
  geometry_msgs::msg::Twist msg;
  float angular_velocity = ANGULAR_VELOCITY;

  if (side_ == RIGHT_SIDE) {
    angular_velocity = -ANGULAR_VELOCITY;
  }

  switch (state_) {
    case GOING_FORWARD:
      msg.linear.x = LINEAL_VELOCITY;
      msg.angular.z = 0;
      RCLCPP_INFO(get_logger(), "GOING_FORWARD");
      break;

    case TURN_SAME_SIDE:
      msg.linear.x = LINEAL_VELOCITY / 3.0;
      msg.angular.z = angular_velocity;
      RCLCPP_INFO(get_logger(), "TURN_SAME_SIDE");
      break;

    case TURN_OPPOSITE_SIDE:
      msg.linear.x = LINEAL_VELOCITY / 3.0;
      msg.angular.z = -angular_velocity;
      RCLCPP_INFO(get_logger(), "TURN_OPPOSITE_SIDE");
      break;

    case WALL_AHEAD:
      msg.linear.x = 0;
      msg.angular.z = -angular_velocity;
      RCLCPP_INFO(get_logger(), "WALL_AHEAD");
      break;
  }

  pubVelocity_->publish(msg);
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Search the wall in the world
void FollowWallNode::LookForWall()
{
  geometry_msgs::msg::Twist msg;

  if (laser_regions[LEFT] < MAX_DISTANCE || laser_regions[CENTER] < MAX_DISTANCE ||
    laser_regions[RIGHT] < MAX_DISTANCE)
  {
    wall_found = true;
    counter_ = 0;
  }

  msg.linear.x = LINEAL_VELOCITY;
  msg.angular.z = 0;

  pubVelocity_->publish(msg);
}

CallbackReturnT FollowWallNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(),
    state.label().c_str());

  pubVelocity_ = this->create_publisher<geometry_msgs::msg::Twist>("/commands/velocity", 100);
  
  state_configure = false;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowWallNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  pubVelocity_->on_activate();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowWallNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
    state.label().c_str());

  pubVelocity_->on_deactivate();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowWallNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(),
    state.label().c_str());
  pubVelocity_.reset();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowWallNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting Down from [%s] state...", get_name(),
    state.label().c_str());

  pubVelocity_.reset();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT FollowWallNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting Down from [%s] state...", get_name(),
    state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

void FollowWallNode::do_work()
{
  
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }
  if (laser_regions.size() > 0) {
    if (pubVelocity_->is_activated()) {
      if(!state_configure){
        SelectSide();
      }
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
