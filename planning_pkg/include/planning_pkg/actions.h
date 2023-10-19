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

// Base class for Cartesian Plan Action Server
class CartesianPlanActionServer
{
public:
    // Constructor for the Cartesian Plan Action Server
    CartesianPlanActionServer(ros::NodeHandle &nh, const std::string &action_name)
        : nh_(nh), action_server_(nh_, action_name, boost::bind(&CartesianPlanActionServer::executeCallback, this, _1), false)
    {
        action_server_.start();
    }

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

    // Helper function to send feedback to the action client
    void sendFeedback(const planning_msgs::CartesianPlanFeedback &feedback)
    {
        if (!preempted_)
        {
            action_server_.publishFeedback(feedback);
        }
    }

    bool isPreemptRequested()
    {
        return action_server_.isPreemptRequested();
    }

    void setPreempted()
    {
        preempted_ = true;
        action_server_.setPreempted();
    }

    // Check if a given pose is really a null pose
    inline bool isNullPose(const geometry_msgs::Pose &pose)
    {
        return (pose.position.x == 0.0 && pose.position.y == 0.0 && pose.position.z == 0.0 &&
                pose.orientation.x == 0.0 && pose.orientation.y == 0.0 && pose.orientation.z == 0.0 && pose.orientation.w == 1.0);
    }

    // Check if a given joint trajectory point is really null
    inline bool isNullPose(const std::vector<double> &joint_configuration)
    {
        return joint_configuration.empty();
        
        // for (auto &q : joint_configuration)
        // {
        //     if (q != 0.0)
        //     {
        //         return false; // Found a non-zero value, not null
        //     }
        // }
        // return true; // All values are zero, considered null
    }
};

// Base class for Joint Plan Action Server
class JointPlanActionServer
{
public:
    // Constructor for the Joint Plan Action Server
    JointPlanActionServer(ros::NodeHandle &nh, const std::string &action_name)
        : nh_(nh), action_server_(nh_, action_name, boost::bind(&JointPlanActionServer::executeCallback, this, _1), false)
    {
        action_server_.start();
    }

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

    // Helper function to send feedback to the action client
    void sendFeedback(const planning_msgs::JointPlanFeedback &feedback)
    {
        if (!preempted_)
        {
            action_server_.publishFeedback(feedback);
        }
    }

    bool isPreemptRequested()
    {
        return action_server_.isPreemptRequested();
    }

    void setPreempted()
    {
        preempted_ = true;
        action_server_.setPreempted();
    }
    // Check if a given joint trajectory point is really null
    inline bool isNullPose(const std::vector<double> &joint_configuration)
    {
        return joint_configuration.empty();
        
        // for (auto &q : joint_configuration)
        // {
        //     if (q != 0.0)
        //     {
        //         return false; // Found a non-zero value, not null
        //     }
        // }
        // return true; // All values are zero, considered null
    }
};


class ExecutePlanActionServer {
public:
    ExecutePlanActionServer(ros::NodeHandle nh,const std::string &action_name) :
        nh_(nh),
        action_server_(nh_, action_name, boost::bind(&ExecutePlanActionServer::executeCallback, this, _1), false)
    {
        action_server_.start();
    }

    void executeCallback(const planning_msgs::ExecutePlanGoalConstPtr &goal);

    void doneCallback(const actionlib::SimpleClientGoalState& state) {
        ROS_INFO("Trajectory execution completed with result: %s", state.toString().c_str());
        // You can perform additional actions or publish feedback here
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<planning_msgs::ExecutePlanAction> action_server_;
};


// Uncomment and implement GraspActionServer as needed
// class GraspActionServer : public ActionServerBase<planning_msgs::GraspAction>
// {
// public:
//     GraspActionServer(ros::NodeHandle &nh, const std::string &action_name)
//         : ActionServerBase<planning_msgs::GraspAction>(nh, action_name)
//     {
//     }

//     void executeCallback(const planning_msgs::GraspGoalConstPtr &goal) override
//     {
//         // Implement the executeCallback specific to GraspAction here
//     }
// };
#endif // ACTIONS_H
