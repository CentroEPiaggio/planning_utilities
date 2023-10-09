#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <planning_msgs/CartesianPlanAction.h>
#include <planning_msgs/JointPlanAction.h>
#include <planning_msgs/GraspAction.h>
#include <planning_msgs/ReleaseAction.h>
#include <Eigen/Dense>

// Define a class template for action servers with feedback, preemption, and goal type
class CartesianPlanActionServer
{
public:
    CartesianPlanActionServer(ros::NodeHandle &nh, const std::string &action_name)
        : nh_(nh), action_server_(nh_, action_name, boost::bind(&CartesianPlanActionServer::executeCallback, this, _1), false)
    {
        action_server_.start();
    }

    virtual void executeCallback(const typename planning_msgs::CartesianPlanGoalConstPtr &goal) = 0;
    ~CartesianPlanActionServer();
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<planning_msgs::CartesianPlanAction> action_server_;
    bool preempted_;

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
    inline bool is_really_null_pose(const geometry_msgs::Pose &pose){
        if (pose.position.x == 0.0 && pose.position.y == 0.0 && pose.position.z == 0.0 &&
            pose.orientation.x == 0.0 && pose.orientation.y == 0.0 && pose.orientation.z == 0.0 && pose.orientation.w == 1.0) {
                return true;
        }

        return false;
    }
};


// // Define specific action server classes using inheritance
// class CartesianPlanActionServer : public ActionServerBase<planning_msgs::CartesianPlanAction>
// {
// public:
//     CartesianPlanActionServer(ros::NodeHandle &nh, const std::string &action_name)
//         : ActionServerBase<planning_msgs::CartesianPlanAction>(nh, action_name)
//     {
//     }

//     void executeCallback(const planning_msgs::CartesianPlanGoalConstPtr &goal);
// private:
//         inline bool is_really_null_pose(const geometry_msgs::Pose &pose){
//             if (pose.position.x == 0.0 && pose.position.y == 0.0 && pose.position.z == 0.0 &&
//                 pose.orientation.x == 0.0 && pose.orientation.y == 0.0 && pose.orientation.z == 0.0 && pose.orientation.w == 1.0) {
//                     return true;
//             }

//             return false;
//         }

// };

// class JointPlanActionServer : public ActionServerBase<planning_msgs::JointPlanAction>
// {
// public:
//     JointPlanActionServer(ros::NodeHandle &nh, const std::string &action_name)
//         : ActionServerBase<planning_msgs::JointPlanAction>(nh, action_name)
//     {
//     }

//     void executeCallback(const planning_msgs::JointPlanGoalConstPtr &goal) override
//     {
//         // Implement the executeCallback specific to JointPlanAction here
//     }
// };

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