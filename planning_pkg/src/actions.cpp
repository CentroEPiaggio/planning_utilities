/**
 * @file actions.cpp
 * @brief Implementation of action server classes for Cartesian and Joint planning.
 *
 * This file contains the implementations of action server classes for Cartesian and Joint planning
 * using the MoveIt MoveGroupInterface. It provides functionality to plan trajectories and respond to
 * action goals.
 *
 * @author Alessandro Palleschi
 * @email alessandropalleschi94@gmail.com
 */

#include "planning_pkg/actions.h"

/**
 * @brief Executes the Cartesian planning action server callback.
 *
 * This function is the callback for the Cartesian planning action server. It extracts the goal
 * information, initializes the MoveGroupInterface, sets the goal pose for planning, and plans the
 * trajectory. The result is set in the action result message.
 *
 * @param goal The Cartesian planning action goal.
 */
void CartesianPlanActionServer::executeCallback(const planning_msgs::CartesianPlanGoalConstPtr &goal)
{
    this->preempted_ = false;

    // Extract goal information
    this->goal_pose = goal->goal_pose;
    this->initial_configuration = goal->initial_configuration;
    this->planning_group = goal->planning_group;

    // Check for preemption before starting any computation
    if (this->isPreemptRequested())
    {
        ROS_INFO("CartesianPlanAction: Preempted");
        setPreempted();
        return;
    }

    // Initialize the MoveGroupInterface and joint model group
    moveit::planning_interface::MoveGroupInterface group(this->planning_group);
    const robot_state::JointModelGroup* joint_model_group =
        group.getCurrentState()->getJointModelGroup(this->planning_group);

    // Set the goal pose for planning
    if (!this->isNullPose(this->goal_pose))
    {
        group.setPoseTarget(this->goal_pose);
    }
    else
    {
        ROS_ERROR("Error: you need to specify a valid pose!!");
        return;
    }

    // Create a start state from the initial configuration if provided
    if (!this->isNullPose(this->initial_configuration))
    {
        moveit::core::RobotStatePtr current_state = group.getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, this->initial_configuration);
        group.setStartState(*current_state);
    }
    else
    {
        // If initial_pose is null, set the start state to the current state
        moveit::core::RobotStatePtr current_state = group.getCurrentState();
        group.setStartState(*current_state); // Use current_state here
    }

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode planning_result = group.plan(plan);

    if (this->isPreemptRequested())
    {
        ROS_INFO("CartesianPlanAction: Preempted");
        setPreempted();
        return;
    }

    // Create the result message
    planning_msgs::CartesianPlanResult result;
    if (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        result.planned_trajectory = plan.trajectory_;
    }
    else
    {
        ROS_ERROR("Planning failed with error code: %d", planning_result.val);
    }

    // Set the action result
    action_server_.setSucceeded(result);
}

/**
 * @brief Executes the Joint planning action server callback.
 *
 * This function is the callback for the Joint planning action server. It extracts the goal
 * information, initializes the MoveGroupInterface, sets the goal pose for planning, and plans the
 * trajectory. The result is set in the action result message.
 *
 * @param goal The Joint planning action goal.
 */
void JointPlanActionServer::executeCallback(const planning_msgs::JointPlanGoalConstPtr &goal)
{
    this->preempted_ = false;

    // Extract goal information
    this->goal_configuration = goal->goal_configuration;
    this->initial_configuration = goal->initial_configuration;
    this->planning_group = goal->planning_group;

    // Check for preemption before starting any computation
    if (this->isPreemptRequested())
    {
        ROS_INFO("JointPlanAction: Preempted");
        this->setPreempted();
        return;
    }

    // Initialize the MoveGroupInterface and joint model group
    moveit::planning_interface::MoveGroupInterface group(this->planning_group);
    const robot_state::JointModelGroup* joint_model_group =
        group.getCurrentState()->getJointModelGroup(this->planning_group);

    // Set the goal pose for planning
    if (!this->isNullPose(this->goal_configuration))
    {
        group.setJointValueTarget(this->goal_configuration);
    }
    else
    {
        return;
    }

    // Create a start state from the initial configuration if provided
    if (!this->isNullPose(this->initial_configuration))
    {
        moveit::core::RobotStatePtr current_state = group.getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, this->initial_configuration);
        group.setStartState(*current_state);
    }
    else
    {
        // If initial_pose is null, set the start state to the current state
        moveit::core::RobotStatePtr current_state = group.getCurrentState();
        group.setStartState(*current_state); // Use current_state here
    }

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode planning_result = group.plan(plan);

    if (this->isPreemptRequested())
    {
        ROS_INFO("JointPlanAction: Preempted");
        this->setPreempted();
        return;
    }

    // Create the result message
    planning_msgs::JointPlanResult result;
    if (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        result.planned_trajectory = plan.trajectory_;
    }
    else
    {
        ROS_ERROR("Planning failed with error code: %d", planning_result.val);
    }

    // Set the action result
    action_server_.setSucceeded(result);
}
