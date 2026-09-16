#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/empty.hpp>

#include <cmath>
#include <vector>

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

using namespace std::placeholders;

class PatrolNodesNode : public rclcpp::Node
{
public:
    PatrolNodesNode() : Node("patrol_node"), current_way_point_(0)
    {
        patrol_nodes_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
        declare_parameter<int>("start_button", 3); //Y
        start_button_ = this->get_parameter("start_button").as_int();
        declare_parameter<int>("cancel_button", 2); //B
        cancel_button_ = this->get_parameter("cancel_button").as_int();
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&PatrolNodesNode::joy_callback, this, _1));

        declare_parameter<std::vector<double>>("waypoints_x",{1.0, 1.0, -3.5, -3.5});
        declare_parameter<std::vector<double>>("waypoints_y",{0.0, 4.0, 4.0, 0.0});
        waypoints_x_ = get_parameter("waypoints_x").as_double_array();
        waypoints_y_ = get_parameter("waypoints_y").as_double_array();

        ball_detector_client_ = this->create_service<std_srvs::srv::Empty>("ball_detection", std::bind(&PatrolNodesNode::ball_detection_callback, this, _1, _2));
    }
private:
    void joy_callback(const sensor_msgs::msg::Joy &msg)
    {
        if (msg.buttons[cancel_button_] == 1 && !previous_cancel_pressed_){
            if (patrol_ongoing_){
                if (patrol_goal_handle_)   patrol_nodes_client_->async_cancel_goal(patrol_goal_handle_);
                else                       RCLCPP_WARN(get_logger(),"Patrol goal has not been accepted yet"); 
            }    
        }
        previous_cancel_pressed_ = msg.buttons[cancel_button_] == 1;

        if (msg.buttons[start_button_] == 1 && !previous_start_pressed_ && !patrol_ongoing_){
            if (!patrol_nodes_client_->action_server_is_ready())
            {
                RCLCPP_WARN(this->get_logger(),"FollowWaypoints action server is not ready");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),"Start partrol process!");
                send_goal();
            }
        }
        previous_start_pressed_ = msg.buttons[start_button_] == 1;
    }

    void send_goal()
    {
        auto goal = FollowWaypoints::Goal();
        goal.number_of_loops = 0;
        goal.goal_index = current_way_point_;

        if (waypoints_x_.size() != waypoints_y_.size() ||  waypoints_x_.size() < 2)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Invalid waypoint parameters");
            return;
        }

        for (size_t i = 0; i < waypoints_x_.size(); i++)
        {
            size_t next = (i + 1) % waypoints_x_.size();
            double yaw = atan2(waypoints_y_[next] - waypoints_y_[i], waypoints_x_[next] - waypoints_x_[i]);
            goal.poses.push_back(Pose2Stamp(waypoints_x_[i], waypoints_y_[i], yaw));
        }

        auto options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        options.goal_response_callback = std::bind(&PatrolNodesNode::goal_response_callback, this, _1);
        options.result_callback = std::bind(&PatrolNodesNode::result_callback, this, _1);
        options.feedback_callback = std::bind(&PatrolNodesNode::feedback_callback, this, _1, _2);


        patrol_nodes_client_->async_send_goal(goal,options);
    }

    void goal_response_callback(const GoalHandleFollow::SharedPtr &goal_handle)
    {
        if (!goal_handle){
            RCLCPP_ERROR(get_logger(),"Patrol goal rejected");
            return;
        }
        patrol_goal_handle_ = goal_handle;
        patrol_ongoing_ = true;
        // RCLCPP_INFO(get_logger(),"Patrol goal accepted");
    }

    void result_callback(const GoalHandleFollow::WrappedResult &result)
    {
        auto status = result.code;
        patrol_goal_handle_.reset();
        if (status == rclcpp_action::ResultCode::SUCCEEDED){
            RCLCPP_INFO(get_logger(),"Patrol goal succeed");
            current_way_point_ = 0;
            send_goal();
        }else if (status == rclcpp_action::ResultCode::CANCELED){
            RCLCPP_INFO(get_logger(),"Patrol goal is canceled");
            patrol_ongoing_ = false;
        }else{
            RCLCPP_INFO(get_logger(),"Patrol goal is aborted");
            patrol_ongoing_ = false;
        }
    }

    void feedback_callback(const std::shared_ptr<GoalHandleFollow> &goal_handle, const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
    {
        (void)goal_handle;
        current_way_point_ = feedback->current_waypoint;
    }


    geometry_msgs::msg::PoseStamped Pose2Stamp(double x, double y, double yaw)
    {
        auto result = geometry_msgs::msg::PoseStamped();
        result.header.frame_id = "map";
        result.header.stamp = now();
        result.pose.position.x = x;
        result.pose.position.y = y;
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        result.pose.orientation = tf2::toMsg(q);
 
        return result;
    }

    void ball_detection_callback(const std_srvs::srv::Empty::Request::SharedPtr request, const std_srvs::srv::Empty::Response::SharedPtr response)
    {
        (void)request;
        (void)response;
        if (!patrol_ongoing_ || !patrol_goal_handle_){
            return;
        }    
        patrol_nodes_client_->async_cancel_goal(patrol_goal_handle_);
        RCLCPP_INFO(get_logger(),"Cancel patroling");
        current_way_point_ ++;
    }

    rclcpp_action::Client<FollowWaypoints>::SharedPtr patrol_nodes_client_;
    GoalHandleFollow::SharedPtr patrol_goal_handle_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;   
    int cancel_button_, start_button_; 
    int current_way_point_;
    bool patrol_ongoing_ = false;
    bool previous_cancel_pressed_ = false;
    bool previous_start_pressed_ = false;
    std::vector<double> waypoints_x_;
    std::vector<double> waypoints_y_;   
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr ball_detector_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolNodesNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}