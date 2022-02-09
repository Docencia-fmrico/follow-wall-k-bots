#ifndef FOLLOW_WALL_HPP

#define FOLLOW_WALL_HPP

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
    CallbackReturnT on_configure(const rclcpp_lifecycle::State &state);
    CallbackReturnT on_activate(const rclcpp_lifecycle::State &state);
    CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &state);
    CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &state);
    CallbackReturnT on_shutdown(const rclcpp_lifecycle::State &state);
    CallbackReturnT on_error(const rclcpp_lifecycle::State &state);
    int angle2pos(float angle, float min, float max, int size);
    void CheckState();
    void FollowTheWall();
    void do_work();
    void LookForWall();
    


private : std::vector<float>
              laser_regions;
std::vector<float> laser_regions_prev;
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;

rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
    pubVelocity_;

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
    TURN_OPPOSITE_SIDE
};

enum laser_side { LEFT = 0, CENTER, RIGHT };

int state_ = GOING_FORWARD;
int side_ = 0;
}
;





#endif  // FOLLOW_WALL_HPP