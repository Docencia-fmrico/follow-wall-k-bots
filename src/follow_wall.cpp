

#include <time.h>

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
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
        laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_raw", 10,
            std::bind(&FollowWallNode::LaserCallback, this, _1));
        pubVelocity_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/nav_vel", 100);

        srand(time(NULL));
    }

    int angle2pos(float angle, float min, float max, int size) {
        return (angle - min) * size / (max - min);
    }

    // Calculate the minimun distances values in the center, right and left of
    // the robot with range

    void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float min = msg->angle_min;
        float max = msg->angle_max;
        float size = msg->ranges.size();

        float center = angle2pos(0, min, max, size);
        float right = angle2pos(-M_PI / 2, min, max, size);
        float left = angle2pos(M_PI / 2, min, max, size);

        float diag_right = angle2pos(-M_PI / 4, min, max, size);
        float diag_left  = angle2pos(M_PI / 4, min, max, size);

        // std::cout << "left = " << left << " center = " << center << " right =
        // " << right << std::endl;
    
        float minRight =
            *std::min_element(std::next(msg->ranges.begin(), right - 1),
                              std::next(msg->ranges.begin(), right + 1));
        float minCenter =
            *std::min_element(std::next(msg->ranges.begin(), center - 1),
                              std::next(msg->ranges.begin(), center + 1));
        float minLeft =
            *std::min_element(std::next(msg->ranges.begin(), left - 1),
                              std::next(msg->ranges.begin(), left + 1));
        
        float minDiagRight =
            *std::min_element(std::next(msg->ranges.begin(), diag_right - 1),
                              std::next(msg->ranges.begin(), diag_right + 1));
        
        float minDiagLeft =
            *std::min_element(std::next(msg->ranges.begin(), diag_left - 1),
                              std::next(msg->ranges.begin(), diag_left + 1));

        std::vector<float> measurements;

        measurements.push_back(minLeft);
        measurements.push_back(minDiagLeft);
        measurements.push_back(minCenter);
        measurements.push_back(minDiagRight);    
        measurements.push_back(minRight);
        
        laser_regions = measurements;
    }

    // Check state depend on the distances
    void CheckState() {
        float side_distance, center_distance;

        center_distance = laser_regions[2];
        if (side_ == LEFT_SIDE) {
            side_distance = laser_regions[0];
        } else {
            side_distance = laser_regions[4];
        }

        if (center_distance > MAX_DISTANCE) {  // Nothing in front
            if (MIN_DISTANCE <= side_distance && side_distance <= MAX_DISTANCE) {
                state_ = GOING_FORWARD;
            } else if (side_distance < MIN_DISTANCE) {  // go futher
                state_ = TURN_OPPOSITE_SIDE;
            } else if (side_distance > MAX_DISTANCE) {  // move closer
                state_ = TURN_SAME_SIDE;
            }
        } else if (MIN_DISTANCE <= center_distance && center_distance <= MAX_DISTANCE){
            if (side_distance < MAX_DISTANCE) {
                state_ = TURN_OPPOSITE_SIDE;
            } else {
                state_ = TURN_SAME_SIDE;
            }
        }
    }

    void FollowTheWall() {
        geometry_msgs::msg::Twist msg;
        float angular_velocity = ANGULAR_VELOCITY;

        if (side_ == LEFT_SIDE) {
            angular_velocity = -ANGULAR_VELOCITY;
        }

        RCLCPP_WARN(get_logger(), "State: %d", state_);

        switch (state_) {
            case GOING_FORWARD:
                msg.linear.x = LINEAL_VELOCITY;
                msg.angular.z = 0;
                break;

            case TURN_SAME_SIDE:
                msg.linear.x = LINEAL_VELOCITY / 4;
                msg.angular.z = angular_velocity;
                break;

            case TURN_OPPOSITE_SIDE:
                msg.linear.x = LINEAL_VELOCITY / 4;
                msg.angular.z = -angular_velocity;
                break;
        }

        pubVelocity_->publish(msg);
    }

    using CallbackReturnT =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /* LookFor Wall in this escenary */
    void LookForWall() {
        geometry_msgs::msg::Twist msg;

        // Case 1: Diagonal la pared
        if (laser_regions[0] < MAX_DISTANCE ||
            laser_regions[2] < MAX_DISTANCE ||
            laser_regions[4] < MAX_DISTANCE) {
            wall_found = true;
        }

        msg.linear.x = LINEAL_VELOCITY;
        msg.angular.z = 0;

        pubVelocity_->publish(msg);
    }

    CallbackReturnT on_configure(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...",
                    get_name(), state.label().c_str());

        // Nos aseguramos que el laser de medidas
        while (laser_regions.size() == 0) {
            continue;
        }

        // Realizamos la configuracion previa para encontrar la pared,
        // comprobando los diferentes casos dependiendo de las medidas de laser

        wall_found = false;
        side_ = RIGHT_SIDE;
        /*
        if (laser_regions[0] < MAX_DISTANCE) {
            side_ = LEFT_SIDE;
            wall_found = true;
        }

        else if (laser_regions[2] < MAX_DISTANCE) {
            side_ = RIGHT_SIDE;
            wall_found = true;
        }

        //Si hemos encontrado la pared decidimos aleatoriamente hacia que lado seguir la pared
        else {
            int num = rand() % 2;

            if (num == 0) {
                side_ = LEFT_SIDE;

            }

            else {
                side_ = RIGHT_SIDE;
            }
        }
        */

        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_activate(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());

        pubVelocity_->on_activate();
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());

        pubVelocity_->on_deactivate();
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...",
                    get_name(), state.label().c_str());
        pubVelocity_.reset();
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_shutdown(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());

        pubVelocity_.reset();
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_error(const rclcpp_lifecycle::State &state) {
        RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...",
                    get_name(), state.label().c_str());
        return CallbackReturnT::SUCCESS;
    }

    void do_work() {
        if (laser_regions.size() > 0) {
            if (get_current_state().id() !=
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
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
            RCLCPP_INFO(get_logger(), "Laser Measures: Left %f, Center %f, Right %f", laser_regions[0], laser_regions[1], laser_regions[2]);
            RCLCPP_INFO(get_logger(), "Node [%s] active", get_name());
        }
    }

   private:
    std::vector<float> laser_regions;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pubVelocity_;

    const float LINEAL_VELOCITY = 0.5;
    const float ANGULAR_VELOCITY = 0.3;
    const float MAX_DISTANCE = 0.52;
    const float MIN_DISTANCE = 0.48;

    bool wall_found = false;

    enum robot_side {
        LEFT_SIDE = 1,
        RIGHT_SIDE
    };

    enum movement {
        GOING_FORWARD = 1,
        TURN_SAME_SIDE,  // wall side
        TURN_OPPOSITE_SIDE
    };

    int state_ = GOING_FORWARD;
    int side_ = 0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FollowWallNode>();

    rclcpp::Rate rate(60);
    while (rclcpp::ok()) {
        node->do_work();

        rclcpp::spin_some(node->get_node_base_interface());
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}