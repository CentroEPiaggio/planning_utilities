#include <ros/ros.h>
#include <XmlRpcValue.h>
#include "planning_pkg/actions.h"
#include <variant>
#include <actionlib/client/simple_action_client.h>

using TaskParameters = std::variant<planning_msgs::CartesianPlanGoal, planning_msgs::JointPlanGoal>;

class TaskHandler {
public:
    TaskHandler(ros::NodeHandle& nh) : nh_(nh) {
        // Initialize action clients
        cartesian_client_.reset(new actionlib::SimpleActionClient<planning_msgs::CartesianPlanAction>("cartesian_plan_action", true));
        joint_client_.reset(new actionlib::SimpleActionClient<planning_msgs::JointPlanAction>("joint_plan_action", true));
        execute_client_.reset(new actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>("execute_plan_action", true));

        // Wait for action servers to start
        if (!cartesian_client_->waitForServer(ros::Duration(5.0)) ||
            !joint_client_->waitForServer(ros::Duration(5.0)) ||
            !execute_client_->waitForServer(ros::Duration(5.0)) {
            ROS_ERROR("One or more action servers did not start within the timeout.");
            ros::shutdown();
        }
    }

    void executePlan(const TaskParameters& goal, bool& is_executing) {
        if (std::holds_alternative<planning_msgs::CartesianPlanGoal>(goal)) {
            planning_msgs::CartesianPlanGoal cartesian_goal = std::get<planning_msgs::CartesianPlanGoal>(goal);
            cartesian_client_->sendGoal(cartesian_goal);
            cartesian_client_->waitForResult();

            if (cartesian_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Cartesian plan succeeded.");
                handleExecution(cartesian_goal.planning_group, *cartesian_client_, is_executing);
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
                handleExecution(joint_goal.planning_group, *joint_client_, is_executing);
            } else {
                ROS_ERROR("Joint plan failed.");
                ros::shutdown();
            }
        }
    }

private:
    ros::NodeHandle& nh_;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::CartesianPlanAction>> cartesian_client_;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::JointPlanAction>> joint_client_;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>> execute_client_;

    void handleExecution(const std::string& planning_group, actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>& client, bool& is_executing) {
        if (is_executing) {
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
            is_executing = true;
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

    bool is_executing = false;

    // Retrieve the task list from the ROS parameter server as XML-RPC data
    XmlRpc::XmlRpcValue task_list;
    if (nh.getParam("/tasks", task_list)) {
        if (task_list.size() > 0) {
            // Iterate over tasks and execute them
            for (int i = 0; i < task_list.size(); ++i) {
                XmlRpc::XmlRpcValue task = task_list[i];
                TaskParameters goal = convertYamlToGoal(task);
                task_handler.executePlan(goal, is_executing);
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
