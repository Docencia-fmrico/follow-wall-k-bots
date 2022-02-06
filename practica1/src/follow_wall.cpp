

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// Execute:
//  ros2 lifecycle list /lifecycle_node_example
//  ros2 lifecycle set /lifecycle_node_example configure

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

class FollowWallNode : public rclcpp_lifecycle::LifecycleNode {
   public:
    FollowWallNode()
        : rclcpp_lifecycle::LifecycleNode("lifecycle_node_example") {
        laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_raw", 10, std::bind(&FollowWallNode::LaserCallback, this, _1));
    }

    int angle2pos(float angle, float min, float max, int size) {
        return (angle - min) * size / (max - min);
    }

    // Calculate the minimun distances values in the center, right and left of the robot
    // with range

    void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float min = msg->angle_min;
        float max = msg->angle_max;
        float size = msg->ranges.size();

        float center = angle2pos(0, min, max, size);
        float right = angle2pos(-M_PI / 2, min, max, size);

        float left = angle2pos(M_PI / 2, min, max, size);

        std::cout << "left = " << left << " center = " << center << " right = " << right << std::endl;

        float minRight = *std::min_element(std::next(msg->ranges.begin(), right - 5), std::next(msg->ranges.begin(), right + 5));
        float minCenter = *std::min_element(std::next(msg->ranges.begin(), center - 5), std::next(msg->ranges.begin(), center + 5));
        float minLeft = *std::min_element(std::next(msg->ranges.begin(), left - 5), std::next(msg->ranges.begin(), left + 5));

        std::cout << "minLeft = " << minLeft << " minCenter = " << minCenter << " minRight = " << minRight << std::endl
                  << std::endl;
    }

    using CallbackReturnT =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturnT on_configure(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());

        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_activate(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_shutdown(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_error(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
        return CallbackReturnT::SUCCESS;
    }

    void do_work() {
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            return;
        }

        RCLCPP_INFO(get_logger(), "Node [%s] active", get_name());
    }

   private:
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan>> laserSub_;
    //rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FollowWallNode>();

    rclcpp::Rate rate(5);
    while (rclcpp::ok()) {
        node->do_work();

        rclcpp::spin_some(node->get_node_base_interface());
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}