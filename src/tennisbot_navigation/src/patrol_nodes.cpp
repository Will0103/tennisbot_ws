#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

using namespace std::placeholders;

class PatrolNodesNode : public rclcpp::Node
{
public:
    PatrolNodesNode() : Node("patrol_node")
    {
        patrol_nodes_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PatrolNodesNode::timer_callback, this));
    }
private:
    void timer_callback()
    {
        
        if (!patrol_nodes_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(get_logger(),
                "FollowWaypoints server not available");
            return;
        }

        timer_->cancel();
        send_goal();
        
    }

    void send_goal()
    {
        auto goal = FollowWaypoints::Goal();
        goal.number_of_loops = 0;
        goal.goal_index = 0;

        double yaw1 = atan2(6.0 - 3.0, -4.0 - 1.0);      // P1 → P2
        double yaw2 = atan2(2.0 - 6.0, -4.0 - (-4.0));   // P2 → P3
        double yaw3 = atan2(0.0 - 2.0, 0.0 - (-4.0));  // P3 → P4
        double yaw4 = atan2(3.0 - 0.0, 1.0 - 0.0); // P4 → P1

        goal.poses.push_back(Pose2Stamp(1.0,3.0, yaw1));
        goal.poses.push_back(Pose2Stamp(-4.0,6.0, yaw2));
        goal.poses.push_back(Pose2Stamp(-4.0,2.0, yaw3));
        goal.poses.push_back(Pose2Stamp(0.0, 0.0, yaw4));

        auto options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        options.goal_response_callback = std::bind(&PatrolNodesNode::goal_response_callback, this, _1);
        options.result_callback = std::bind(&PatrolNodesNode::result_callback, this, _1);


        patrol_nodes_client_->async_send_goal(goal,options);
    }

    void goal_response_callback(const GoalHandleFollow::SharedPtr &goal_handle)
    {
        if (!goal_handle){
            RCLCPP_ERROR(get_logger(),"Patrol goal rejected");
            return;
        }
        patrol_goal_handle_ = goal_handle;
        RCLCPP_INFO(get_logger(),"Patrol goal accepted");
    }

    void result_callback(const GoalHandleFollow::WrappedResult &result)
    {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED){
            RCLCPP_INFO(get_logger(),"Patrol goal succeed");
            send_goal();
        }else if (status == rclcpp_action::ResultCode::CANCELED){
            RCLCPP_INFO(get_logger(),"Patrol goal is canceled");
        }else{
            RCLCPP_INFO(get_logger(),"Patrol goal is aborted");
        }
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



    rclcpp_action::Client<FollowWaypoints>::SharedPtr patrol_nodes_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    GoalHandleFollow::SharedPtr patrol_goal_handle_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolNodesNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}