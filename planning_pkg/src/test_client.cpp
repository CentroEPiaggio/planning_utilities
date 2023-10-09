#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <planning_msgs/JointPlanAction.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_plan_client");
    ros::NodeHandle nh;

    // Create an Action client for the JointPlan action server
    actionlib::SimpleActionClient<planning_msgs::JointPlanAction> joint_plan_client("joint_plan_action", true);

    // Wait for the action server to start
    ROS_INFO("Waiting for JointPlan action server to start...");
    joint_plan_client.waitForServer();

    // Create a goal to send to the action server
    planning_msgs::JointPlanGoal goal;
    // Load goal configuration from the parameter server
    if (!ros::param::get("/joint_test_client/goal_configuration", goal.goal_configuration))
    {
        ROS_ERROR("Failed to load goal_configuration parameter.");
        return 1;
    }

    // Load initial configuration from the parameter server
    if (!ros::param::get("/joint_test_client/initial_configuration", goal.initial_configuration))
    {
        ROS_ERROR("Failed to load initial_configuration parameter.");
        return 1;
    }

    // Load planning group from the parameter server
    if (!ros::param::get("/joint_test_client/planning_group", goal.planning_group))
    {
        ROS_ERROR("Failed to load planning_group parameter.");
        return 1;
    }
    // Send the goal to the action server
    ROS_INFO("Sending goal to JointPlan action server...");
    joint_plan_client.sendGoal(goal);

    // Wait for the action to complete (you can add a timeout here)
    bool finished_before_timeout = joint_plan_client.waitForResult();

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = joint_plan_client.getState();
        ROS_INFO("JointPlan action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_ERROR("JointPlan action did not complete before the timeout.");
    }

    return 0;
}
