/**
 * @file actions.h
 * @brief Header file for various ROS action servers.
 * @author Alessandro Palleschi
 */

#ifndef ACTIONS_H
#define ACTIONS_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <planning_msgs/CartesianPlanAction.h>
#include <planning_msgs/JointPlanAction.h>
#include <planning_msgs/GraspAction.h>
#include <planning_msgs/ReleaseAction.h>
#include <planning_msgs/ExecutePlanAction.h>
#include <Eigen/Dense>

/**
 * @class CartesianPlanActionServer
 * @brief Base class for Cartesian Plan Action Server.
 */
class CartesianPlanActionServer
{
public:
    /**
     * @brief Constructor for the Cartesian Plan Action Server.
     * @param nh The ROS NodeHandle.
     * @param action_name The name of the action server.
     */
    CartesianPlanActionServer(ros::NodeHandle &nh, const std::string &action_name)
        : nh_(nh), action_server_(nh_, action_name, boost::bind(&CartesianPlanActionServer::executeCallback, this, _1), false)
    {
        action_server_.start();
    }

    /**
     * @brief Callback function for executing the Cartesian Plan action.
     * @param goal The goal message for the action.
     */
    void executeCallback(const planning_msgs::CartesianPlanGoalConstPtr &goal);

    ~CartesianPlanActionServer() {};

protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<planning_msgs::CartesianPlanAction> action_server_;
    bool preempted_;

    // Present joint configuration and computed trajectory
    std::vector<double> initial_configuration;
    geometry_msgs::Pose goal_pose;
    std::string planning_group;
    trajectory_msgs::JointTrajectory computed_trajectory;

    /**
     * @brief Helper function to send feedback to the action client.
     * @param feedback The feedback message to be sent.
     */
    void sendFeedback(const planning_msgs::CartesianPlanFeedback &feedback)
    {
        if (!preempted_)
        {
            action_server_.publishFeedback(feedback);
        }
    }

    /**
     * @brief Check if a given pose is a null pose.
     * @param pose The pose to check.
     * @return True if the pose is null, false otherwise.
     */
    inline bool isNullPose(const geometry_msgs::Pose &pose)
    {
        return (pose.position.x == 0.0 && pose.position.y == 0.0 && pose.position.z == 0.0 &&
                pose.orientation.x == 0.0 && pose.orientation.y == 0.0 && pose.orientation.z == 0.0 && pose.orientation.w == 1.0);
    }

    /**
     * @brief Check if a given joint trajectory point is null.
     * @param joint_configuration The joint configuration to check.
     * @return True if the joint configuration is null, false otherwise.
     */
    inline bool isNullPose(const std::vector<double> &joint_configuration)
    {
        return joint_configuration.empty();
    }
};

/**
 * @class JointPlanActionServer
 * @brief Base class for Joint Plan Action Server.
 */
class JointPlanActionServer
{
public:
    /**
     * @brief Constructor for the Joint Plan Action Server.
     * @param nh The ROS NodeHandle.
     * @param action_name The name of the action server.
     */
    JointPlanActionServer(ros::NodeHandle &nh, const std::string &action_name)
        : nh_(nh), action_server_(nh_, action_name, boost::bind(&JointPlanActionServer::executeCallback, this, _1), false)
    {
        action_server_.start();
    }

    /**
     * @brief Callback function for executing the Joint Plan action.
     * @param goal The goal message for the action.
     */
    void executeCallback(const typename planning_msgs::JointPlanGoalConstPtr &goal);
    ~JointPlanActionServer() {};

protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<planning_msgs::JointPlanAction> action_server_;
    bool preempted_;

    // Present joint configuration and computed trajectory
    std::vector<double> initial_configuration;
    std::vector<double> goal_configuration;
    std::string planning_group;

    trajectory_msgs::JointTrajectory computed_trajectory;

    /**
     * @brief Helper function to send feedback to the action client.
     * @param feedback The feedback message to be sent.
     */
    void sendFeedback(const planning_msgs::JointPlanFeedback &feedback)
    {
        if (!preempted_)
        {
            action_server_.publishFeedback(feedback);
        }
    }

    /**
     * @brief Check if a given joint trajectory point is null.
     * @param joint_configuration The joint configuration to check.
     * @return True if the joint configuration is null, false otherwise.
     */
    inline bool isNullPose(const std::vector<double> &joint_configuration)
    {
        return joint_configuration.empty();
    }
};

/**
 * @class ExecutePlanActionServer
 * @brief Class for executing a plan using an action server.
 */
class ExecutePlanActionServer {
public:
    /**
     * @brief Constructor for the Execute Plan Action Server.
     * @param nh The ROS NodeHandle.
     * @param action_name The name of the action server.
     */
    ExecutePlanActionServer(ros::NodeHandle nh, const std::string &action_name) :
        nh_(nh),
        action_server_(nh_, action_name, boost::bind(&ExecutePlanActionServer::executeCallback, this, _1), false)
    {
        action_server_.start();
    }

    /**
     * @brief Callback function for executing the Execute Plan action.
     * @param goal The goal message for the action.
     */
    void executeCallback(const planning_msgs::ExecutePlanGoalConstPtr &goal);

    /**
     * @brief Callback function for handling the completion of the trajectory execution.
     * @param state The state of the goal execution.
     */
    void doneCallback(const actionlib::SimpleClientGoalState& state) {
        ROS_INFO("Trajectory execution completed with result: %s", state.toString().c_str());
        // You can perform additional actions or publish feedback here
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<planning_msgs::ExecutePlanAction> action_server_;
};

/**
 * Uncomment and implement GraspActionServer as needed.
 */
// class GraspActionServer : public ActionServerBase<planning_msgs::GraspAction>
// {
// public:
//     GraspActionServer(ros::NodeHandle &nh, const std::string &action_name)
//         : ActionServerBase<planning_msgs::GraspAction>(nh, action_name)
//     {
//     }
//
//     void executeCallback(const planning_msgs::GraspGoalConstPtr &goal) override
//     {
//         // Implement the executeCallback specific to GraspAction here
//     }
// };

#endif // ACTIONS_H
