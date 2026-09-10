#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::placeholders;

class JoystickFunctionNode : public rclcpp::Node
{
public:
    JoystickFunctionNode() : Node("joystick_function")
    {
        declare_parameter<int>("cancel_button", 2);
        cancel_button_ = this->get_parameter("cancel_button").as_int();
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickFunctionNode::joy_callback, this, _1));
        cancel_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    }
private:
    void joy_callback(const sensor_msgs::msg::Joy &msg)
    {
        bool pressed = msg.buttons[cancel_button_] == 1;

        if (pressed && !previous_pressed_){
            if (!cancel_client_->action_server_is_ready())
            {
                RCLCPP_WARN(this->get_logger(),"NavigateToPose action server is not ready");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),"Cancel navigation process!");
                cancel_client_->async_cancel_all_goals();
            }
        }
        previous_pressed_ = pressed;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;    
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr cancel_client_;
    int cancel_button_;
    bool previous_pressed_ = false;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickFunctionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}