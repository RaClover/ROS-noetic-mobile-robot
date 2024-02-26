#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv) {
    // Connect to ROS
    ros::init(argc, argv, "simple_navigation_goals");

    // Create an action client
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    // Wait for the action server to come up
    if (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Failed to connect to the move_base action server");
        return 1;
    }

    // Create a new goal to send to move_base (Setting a single goal, House 1)
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    cout << "Setting a single goal: House 1\n";
    goal.target_pose.pose.position.x = -15.04;
    goal.target_pose.pose.position.y = -7.42;

    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait for the result with a timeout
    if (ac.waitForResult(ros::Duration(180.0))) {
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The robot has arrived at the goal location");
        else
            ROS_WARN("The robot may have failed to reach the goal location");
    } else {
        ROS_ERROR("Failed to get the result within the specified timeout");
        return 1;
    }

    return 0;
}
