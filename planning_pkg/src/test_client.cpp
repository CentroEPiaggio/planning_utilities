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
    // Fill in the goal with the desired joint trajectory or configuration
    goal.goal_configuration = {1.0, 1.0, 0.0, 0.2, 1.3, 1.57, 0, 0.06};
    // goal.initial_configuration = {-1.0, 1.0, 0.0, 0.2, 1.3, 1.57, 0};
    goal.initial_configuration = {-0.5, -0.5374493632429661, 0.06520185673784341, -2.189379671612611, 0.033515001883730244, 1.6528250384317584, 0.9420949765761668-1.0, 1.0, 0.0, 0.2, 1.3, 1.57, 0, 0};

    goal.planning_group = "panda_manipulator";
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
