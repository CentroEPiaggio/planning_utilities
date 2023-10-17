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
    /**
     * @brief Constructor for TaskHandler.
     * 
     * @param nh Reference to the ROS NodeHandle.
     */
    TaskHandler(ros::NodeHandle& nh)
        : nh_(nh),
          is_executing_(false),
          // Initialize action clients for Cartesian and Joint planning
          cartesian_client_(new actionlib::SimpleActionClient<planning_msgs::CartesianPlanAction>("cartesian_plan_action", true)),
          joint_client_(new actionlib::SimpleActionClient<planning_msgs::JointPlanAction>("joint_plan_action", true)),
          execute_client_(new actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>("execute_plan_action", true)) {

        // Wait for action servers to start
        if (!cartesian_client_->waitForServer(ros::Duration(5.0)) ||
            !joint_client_->waitForServer(ros::Duration(5.0)) ||
            !execute_client_->waitForServer(ros::Duration(5.0)) {
            ROS_ERROR("One or more action servers did not start within the timeout.");
            ros::shutdown();
        }
    }

    /**
     * @brief Execute a task based on the provided goal.
     * 
     * @param goal Task parameters as a variant type.
     */
    void executePlan(const TaskParameters& goal) {
        if (std::holds_alternative<planning_msgs::CartesianPlanGoal>(goal)) {
            planning_msgs::CartesianPlanGoal cartesian_goal = std::get<planning_msgs::CartesianPlanGoal>(goal);
            // Send Cartesian planning goal to the action server
            cartesian_client_->sendGoal(cartesian_goal);
            cartesian_client_->waitForResult();

            if (cartesian_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Cartesian plan succeeded.");
                // Handle execution if it's in progress or set a new goal
                handleExecution(cartesian_goal.planning_group, *cartesian_client_);
            } else {
                ROS_ERROR("Cartesian plan failed.");
                ros::shutdown();
            }
        } else if (std::holds_alternative<planning_msgs::JointPlanGoal>(goal)) {
            planning_msgs::JointPlanGoal joint_goal = std::get<planning_msgs::JointPlanGoal>(goal);
            // Send Joint planning goal to the action server
            joint_client_->sendGoal(joint_goal);
            joint_client_->waitForResult();

            if (joint_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Joint plan succeeded.");
                // Handle execution if it's in progress or set a new goal
                handleExecution(joint_goal.planning_group, *joint_client_);
            } else {
                ROS_ERROR("Joint plan failed.");
                ros::shutdown();
            }
        }
    }

private:
    ros::NodeHandle& nh_;
    bool is_executing_;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::CartesianPlanAction>> cartesian_client_;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::JointPlanAction>> joint_client_;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>> execute_client_;

    /**
     * @brief Handle the execution of a planned task.
     * 
     * @param planning_group The name of the planning group.
     * @param client The action client for execution.
     */
    void handleExecution(const std::string& planning_group, actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>& client) {
        if (is_executing_) {
            client.waitForResult();
            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                planning_msgs::ExecutePlanGoal execute_goal;
                execute_goal.move_group_name = planning_group;
                execute_goal.motion_plan = client.getResult()->planned_trajectory;
                // Send the execution goal to the action server
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
            // Send the execution goal to the action server
            execute_client_->sendGoal(execute_goal);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_planner_node");
    ros::NodeHandle nh;

    TaskHandler task_handler(nh);

    // Retrieve the task list from the ROS parameter server as XML-RPC data
    XmlRpc::XmlRpcValue task_list;
    if (nh.getParam("/tasks", task_list)) {
        if (task_list.size() > 0) {
            // Iterate over tasks and execute them
            for (int i = 0; i < task_list.size(); ++i) {
                XmlRpc::XmlRpcValue task = task_list[i];
                TaskParameters goal = convertYamlToGoal(task);
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
