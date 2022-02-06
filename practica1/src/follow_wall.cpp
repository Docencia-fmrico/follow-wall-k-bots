

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Execute:
//  ros2 lifecycle list /lifecycle_node_example
//  ros2 lifecycle set /lifecycle_node_example configure

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

class LifeCycleNodeExample : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifeCycleNodeExample()
        : rclcpp_lifecycle::LifecycleNode("lifecycle_node_example")
    {

    }

    using CallbackReturnT =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturnT on_configure(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());

        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
        return CallbackReturnT::SUCCESS;
    }

    CallbackReturnT on_error(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
        return CallbackReturnT::SUCCESS;
    }

    void do_work()
    {
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            return;
        }

        RCLCPP_INFO(get_logger(), "Node [%s] active", get_name());
    }

private:
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LifeCycleNodeExample>();

    rclcpp::Rate rate(5);
    while (rclcpp::ok())
    {
        node->do_work();



        rclcpp::spin_some(node->get_node_base_interface());
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}