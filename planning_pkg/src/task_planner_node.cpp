#include <ros/ros.h>
#include <XmlRpcValue.h>
#include "planning_pkg/actions.h"
#include <variant>
#include <actionlib/client/simple_action_client.h>

// Define a variant type for task parameters
using TaskParameters = std::variant<planning_msgs::CartesianPlanGoal, planning_msgs::JointPlanGoal>;

/**
 * @brief Class to handle task execution using ROS action clients.
 */
class TaskHandler {
public:
    // Constructor with initialization list
    TaskHandler(ros::NodeHandle& nh)
        : nh_(nh),
          is_executing_(false),
          cartesian_client_(new actionlib::SimpleActionClient<planning_msgs::CartesianPlanAction>("cartesian_plan_action", true)),
          joint_client_(new actionlib::SimpleActionClient<planning_msgs::JointPlanAction>("joint_plan_action", true)),
          execute_client_(new actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>("execute_plan_action", true)) {
        // Wait for action servers to start
        if (!cartesian_client_->waitForServer(ros::Duration(5.0)) ||
            !joint_client_->waitForServer(ros::Duration(5.0)) ||
            !execute_client_->waitForServer(ros::Duration(5.0))) {
            ROS_ERROR("One or more action servers did not start within the timeout.");
            ros::shutdown();
        }
    }

    /**
     * @brief Parse a Pose from XML-RPC data.
     * 
     * @param pose_param XML-RPC data containing pose information.
     * @return Parsed geometry_msgs::Pose.
     */
    geometry_msgs::Pose parsePose(const XmlRpc::XmlRpcValue& pose_param) {
        geometry_msgs::Pose pose;
        pose.position.x = static_cast<double>(pose_param["position"]["x"]);
        pose.position.y = static_cast<double>(pose_param["position"]["y"]);
        pose.position.z = static_cast<double>(pose_param["position"]["z"]);
        pose.orientation.x = static_cast<double>(pose_param["orientation"]["x"]);
        pose.orientation.y = static_cast<double>(pose_param["orientation"]["y"]);
        pose.orientation.z = static_cast<double>(pose_param["orientation"]["z"]);
        pose.orientation.w = static_cast<double>(pose_param["orientation"]["w"]);
        return pose;
    }

    /**
     * @brief Convert YAML data to a task parameter variant.
     * 
     * @param task_param XML-RPC data containing task parameters.
     * @return TaskParameters variant representing the task goal.
     */
    TaskParameters convertYamlToGoal(const XmlRpc::XmlRpcValue& task_param) {
        if (task_param.hasMember("type")) {
            std::string task_type = static_cast<std::string>(task_param["type"]);

            if (task_type == "CartesianPlan") {
                planning_msgs::CartesianPlanGoal cartesian_goal;

                if (task_param.hasMember("goal")) {
                    cartesian_goal.goal_pose = parsePose(task_param["goal"]);
                }

                if (task_param.hasMember("initial") && task_param["initial"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                    for (int i = 0; i < task_param["initial"].size(); ++i) {
                        if (task_param["initial"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                            double value = static_cast<double>(task_param["initial"][i]);
                            cartesian_goal.initial_configuration.push_back(value);
                        }
                    }
                }

                if (task_param.hasMember("group")) {
                    cartesian_goal.planning_group = static_cast<std::string>(task_param["group"]);
                }

                return cartesian_goal;
            } else if (task_type == "JointPlan") {
                planning_msgs::JointPlanGoal joint_goal;

                if (task_param.hasMember("goal")) {
                    XmlRpc::XmlRpcValue goal = task_param["goal"];
                    for (int i = 0; i < goal.size(); ++i) {
                        if (goal[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                            double value = static_cast<double>(goal[i]);
                            joint_goal.goal_configuration.push_back(value);
                        }
                    }
                }

                if (task_param.hasMember("initial") && task_param["initial"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                    for (int i = 0; i < task_param["initial"].size(); ++i) {
                        if (task_param["initial"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                            double value = static_cast<double>(task_param["initial"][i]);
                            joint_goal.initial_configuration.push_back(value);
                        }
                    }
                }

                if (task_param.hasMember("group")) {
                    joint_goal.planning_group = static_cast<std::string>(task_param["group"]);
                }

                return joint_goal;
            }
        }

        ROS_ERROR("Wrong task parameters!");
        return TaskParameters(); // Return an empty variant in case of an error
    }

    /**
     * @brief Execute a task based on the provided goal.
     * 
     * @param goal Task parameters as a variant type.
     */
    void executePlan(const TaskParameters& goal) {
        if (std::holds_alternative<planning_msgs::CartesianPlanGoal>(goal)) {
            planning_msgs::CartesianPlanGoal cartesian_goal = std::get<planning_msgs::CartesianPlanGoal>(goal);
            cartesian_client_->sendGoal(cartesian_goal);
            cartesian_client_->waitForResult();

            if (cartesian_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Cartesian plan succeeded.");
                handleExecution(cartesian_goal.planning_group, *cartesian_client_);
            } else {
                ROS_ERROR("Cartesian plan failed.");
                ros::shutdown();
            }
        } else if (std::holds_alternative<planning_msgs::JointPlanGoal>(goal)) {
            planning_msgs::JointPlanGoal joint_goal = std::get<planning_msgs::JointPlanGoal>(goal);
            joint_client_->sendGoal(joint_goal);
            joint_client_->waitForResult();

            if (joint_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Joint plan succeeded.");
                handleExecution(joint_goal.planning_group, *joint_client_);
            } else {
                ROS_ERROR("Joint plan failed.");
                ros::shutdown();
            }
        } else {
                ROS_ERROR("Unexpected Plan TYPE!");
                ros::shutdown();
        }

    }

private:
    ros::NodeHandle& nh_;
    bool is_executing_;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::CartesianPlanAction>> cartesian_client_;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::JointPlanAction>> joint_client_;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>> execute_client_;

    void handleExecution(const std::string& planning_group, actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>& client) {
        if (is_executing_) {
            client.waitForResult();
            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                planning_msgs::ExecutePlanGoal execute_goal;
                execute_goal.move_group_name = planning_group;
                execute_goal.motion_plan = client.getResult()->planned_trajectory;
                execute_client_->sendGoal(execute_goal);
            } else {
                ROS_ERROR("Execution FAILED!");
                ros::shutdown();
            }
        } else {
            is_executing_ = true;
            planning_msgs::ExecutePlanGoal execute_goal;
            execute_goal.move_group_name = planning_group;
            execute_goal.motion_plan = client.getResult()->planned_trajectory;
            execute_client_->sendGoal(execute_goal);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_planner_node");
    ros::NodeHandle nh;

    TaskHandler task_handler(nh);

    XmlRpc::XmlRpcValue task_list;
    if (nh.getParam("/tasks", task_list)) {
        if (task_list.size() > 0) {
            for (int i = 0; i < task_list.size(); ++i) {
                XmlRpc::XmlRpcValue task = task_list[i];
                TaskParameters goal = task_handler.convertYamlToGoal(task);
                task_handler.executePlan(goal);
            }
        } else {
            ROS_ERROR("Task List is Empty!");
            return 1;
        }
    } else {
        ROS_ERROR("Failed to retrieve the /task_list parameter from the parameter server.");
        return 1;
    }
    return 0;
}
