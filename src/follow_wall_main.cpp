
#include "follow_wall/follow_wall.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FollowWallNode>();

    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
        node->do_work();

        rclcpp::spin_some(node->get_node_base_interface());
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}


