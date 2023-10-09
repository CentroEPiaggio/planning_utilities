/* Cartesina Plan - Uses moveit movegroupinterface to plan towards a Cartesian pose
Author: Alessandro Palleschi
Email: alessandropalleschi94@gmail.com  */

#include "planning_pkg/actions.h"

void CartesianPlanActionServer::executeCallback(const planning_msgs::CartesianPlanGoalConstPtr &goal)
{
    preempted_ = false;

    // Extract goal information
    geometry_msgs::Pose goal_pose = goal->goal_pose;
    geometry_msgs::Pose initial_pose = goal->initial_pose;
    std::string planning_group = goal->planning_group;

    // Check for preemption before starting any computation
    if (isPreemptRequested())
    {
        ROS_INFO("CartesianPlanAction: Preempted");
        setPreempted();
        return;
    }

    // Initialize the MoveGroupInterface and joint model group
    moveit::planning_interface::MoveGroupInterface group(planning_group);
    const robot_state::JointModelGroup* joint_model_group =
        group.getCurrentState()->getJointModelGroup(planning_group);

    // Set the goal pose for planning
    if (!this->is_really_null_pose(goal_pose))
    {
        group.setPoseTarget(goal_pose);
    }
    else
    {
        return;
    }

    // Create a start state from the initial pose if provided
    if (!this->is_really_null_pose(initial_pose))
    {
        robot_state::RobotState start_state(*group.getCurrentState());
        start_state.setFromIK(joint_model_group, initial_pose);
        group.setStartState(start_state);
    }

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode planning_result = group.plan(plan);

    if (isPreemptRequested())
    {
        ROS_INFO("CartesianPlanAction: Preempted");
        setPreempted();
        return;
    }

    // Create the result message
    planning_msgs::CartesianPlanResult result;
    if (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        // moveit_msgs::RobotTrajectory trajectory;
        // plan.trajectory_->getRobotTrajectoryMsg(trajectory);
        result.planned_trajectory = plan.trajectory_;
    }
    else
    {
        ROS_ERROR("Planning failed with error code: %d", planning_result.val);
    }

    // Set the action result
    action_server_.setSucceeded(result);
}