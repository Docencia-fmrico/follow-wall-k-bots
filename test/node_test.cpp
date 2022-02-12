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

#include <memory>

#include "follow_wall/follow_wall.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

TEST(node_test, test_angle2pos) {
  auto node = std::make_shared<FollowWallNode>();

  ASSERT_EQ(node->angle2pos(0, -M_PI_2, M_PI_2, 5), 2);
  ASSERT_EQ(node->angle2pos(-M_PI_2, -M_PI_2, M_PI_2, 5), 0);
  ASSERT_EQ(node->angle2pos(M_PI_2, -M_PI_2, M_PI_2, 5), 4);
  ASSERT_EQ(node->angle2pos(M_PI_4, -M_PI_2, M_PI_2, 5), 3);
  ASSERT_EQ(node->angle2pos(-M_PI_4, -M_PI_2, M_PI_2, 5), 1);
  ASSERT_EQ(node->angle2pos(-M_PI_4, -M_PI_2, M_PI_2, 180), 45);
  ASSERT_EQ(node->angle2pos(-M_PI_4, -M_PI, M_PI, 360), 135);
}

TEST(node_test, test_checkState) {
  sensor_msgs::msg::LaserScan msg;
  auto node = std::make_shared<FollowWallNode>();
  auto node_test = rclcpp::Node::make_shared("test_node");
  auto publisher = node_test->create_publisher<sensor_msgs::msg::LaserScan>("/scan_raw", 10);

  std::cout << "PEPE 1" << std::endl;
  msg.angle_min = -M_PI;
  std::cout << "PEPE 2" << std::endl;
  msg.angle_max = M_PI;
  std::cout << "PEPE 3" << std::endl;
  msg.ranges = std::vector<float>(180, 10);

  msg.ranges[45] = 5;
  msg.ranges[90] = 6;
  msg.ranges[135] = 3;

  std::thread t([&]() {
    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
      node->do_work();

      rclcpp::spin_some(node->get_node_base_interface());
      rate.sleep();
    }
  });

  std::vector<float> laser_regions = node->getLaserRegions();

  while (laser_regions.size() == 0) {
    laser_regions = node->getLaserRegions();
  }

  ASSERT_EQ(laser_regions[0], 3);
  ASSERT_EQ(laser_regions[1], 6);
  ASSERT_EQ(laser_regions[2], 5);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
