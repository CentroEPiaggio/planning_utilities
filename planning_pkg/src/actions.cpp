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
    planning_msgs::CartesianPlanResult result;

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
    auto joint_names = group.getJointNames();

    // Set the goal pose for planning
    if (!this->isNullPose(this->goal_pose))
    {
        group.setPoseTarget(this->goal_pose);
    }
    else
    {
        ROS_ERROR("Empty goal configuration");
        action_server_.setAborted(result);
        return;
    }

    // Create a start state from the initial configuration if provided
    if (!this->isNullPose(this->initial_configuration))
    {
        ROS_INFO("Setting Initial Position");
        if(joint_names.size() == this->initial_configuration.size())
        {
            moveit::core::RobotStatePtr current_state = group.getCurrentState();
            current_state->setJointGroupPositions(this->planning_group,this->initial_configuration);
            group.setStartState(*current_state);
        }
          else {
            ROS_ERROR("WRONG SIZE: the number of joints in the goal is %zu, while it should be %zu", initial_configuration.size(), joint_names.size());
            action_server_.setAborted(result);
            return;

        }

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
    if (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        result.planned_trajectory = plan.trajectory_;
    }
    else
    {
        action_server_.setAborted(result);
        ROS_ERROR("Planning failed with error code: %d", planning_result.val);
        return;

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
    for(auto &e : this->initial_configuration) std::cout << e << std::endl;

    // Check for preemption before starting any computation
    if (this->isPreemptRequested())
    {
        ROS_INFO("JointPlanAction: Preempted");
        this->setPreempted();
        return;
    }
    // Create the result message
    planning_msgs::JointPlanResult result;

    // Initialize the MoveGroupInterface and joint model group
    moveit::planning_interface::MoveGroupInterface group(this->planning_group);
    const robot_state::JointModelGroup* joint_model_group =
        group.getCurrentState()->getJointModelGroup(this->planning_group);
    auto joint_names = group.getJointNames();
    // Set the goal pose for planning
    if (!this->isNullPose(this->goal_configuration))
    {
        if(joint_names.size() == this->goal_configuration.size()) group.setJointValueTarget(this->goal_configuration);
        else {
            ROS_ERROR("WRONG SIZE: the number of joints in the goal is %zu, while it should be %zu", goal_configuration.size(), joint_names.size());
            action_server_.setAborted(result);
            return;

        }
    }
    else
    {
        ROS_ERROR("Empty goal configuration");
        action_server_.setAborted(result);
        return;
    }

    // Create a start state from the initial configuration if provided
    if (!this->isNullPose(this->initial_configuration))
    {
        ROS_INFO("Setting Initial Position");
        if(joint_names.size() == this->initial_configuration.size())
        {
            moveit::core::RobotStatePtr current_state = group.getCurrentState();
            current_state->setJointGroupPositions(this->planning_group,this->initial_configuration);
            group.setStartState(*current_state);
        }
          else {
            ROS_ERROR("WRONG SIZE: the number of joints in the goal is %zu, while it should be %zu", initial_configuration.size(), joint_names.size());
            action_server_.setAborted(result);
            return;

        }

    }
    else
    {
        // If initial_pose is null, set the start state to the current state
        ROS_INFO("No intial position specified, using the current state");
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

    if (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        result.planned_trajectory = plan.trajectory_;
    }
    else
    {
        action_server_.setAborted(result);
        ROS_ERROR("Planning failed with error code: %d", planning_result.val);
        return;
    }

    // Set the action result
    action_server_.setSucceeded(result);
}
