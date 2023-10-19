// #include <ros/ros.h>
// #include <XmlRpcValue.h>
// #include "planning_pkg/actions.h"
// #include <variant>
// #include <queue>
// #include <actionlib/client/simple_action_client.h>
// #include <thread>
// #include <chrono>

// /**
//  * @brief Class to handle task execution using ROS action clients.
//  */
// class TaskHandler {
// public:
//     /**
//      * @brief Constructor for TaskHandler.
//      *
//      * @param nh Reference to the ROS NodeHandle.
//      */
//     TaskHandler(ros::NodeHandle& nh)
//         : nh_(nh),
//           is_executing_(false),
//           last_planned_configuration_(nullptr), // Initialize as nullptr
//           cartesian_client_(new actionlib::SimpleActionClient<planning_msgs::CartesianPlanAction>("cartesian_plan_action", true)),
//           joint_client_(new actionlib::SimpleActionClient<planning_msgs::JointPlanAction>("joint_plan_action", true)),
//           execute_client_(new actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>("execute_plan_action", true)) {
//         // Wait for action servers to start
//         if (!cartesian_client_->waitForServer(ros::Duration(5.0)) ||
//             !joint_client_->waitForServer(ros::Duration(5.0)) ||
//             !execute_client_->waitForServer(ros::Duration(5.0))) {
//             ROS_ERROR("One or more action servers did not start within the timeout.");
//             ros::shutdown();
//         }
//         execution_thread_ = std::thread(&TaskHandler::handleExecution, this);
//     }

//     /**
//      * @brief Destructor for TaskHandler, joins the execution thread.
//      */
//     ~TaskHandler() {
//         stopExecutionThread();
//     }

//     /**
//      * @brief Parse a Pose from XML-RPC data.
//      *
//      * @param pose_param XML-RPC data containing pose information.
//      * @return Parsed geometry_msgs::Pose.
//      */
//     geometry_msgs::Pose parsePose(const XmlRpc::XmlRpcValue& pose_param) {
//         geometry_msgs::Pose pose;
//         pose.position.x = static_cast<double>(pose_param["position"]["x"]);
//         pose.position.y = static_cast<double>(pose_param["position"]["y"]);
//         pose.position.z = static_cast<double>(pose_param["position"]["z"]);
//         pose.orientation.x = static_cast<double>(pose_param["orientation"]["x"]);
//         pose.orientation.y = static_cast<double>(pose_param["orientation"]["y"]);
//         pose.orientation.z = static_cast<double>(pose_param["orientation"]["z"]);
//         pose.orientation.w = static_cast<double>(pose_param["orientation"]["w"]);
//         return pose;
//     }

//     /**
//      * @brief Convert YAML data to a task parameter variant.
//      *
//      * @param task_param XML-RPC data containing task parameters.
//      * @return PlanningGoal variant representing the task goal.
//      */
//     PlanningGoal convertYamlToGoal(const XmlRpc::XmlRpcValue& task_param) {
//         if (task_param.hasMember("type")) {
//             std::string task_type = static_cast<std::string>(task_param["type"]);

//             if (task_type == "CartesianPlan") {
//                 planning_msgs::CartesianPlanGoal cartesian_goal;

//                 if (task_param.hasMember("goal")) {
//                     cartesian_goal.goal_pose = parsePose(task_param["goal"]);
//                 }
//                 else {
//                     ROS_ERROR("No goal specified!");
//                     return std::monostate{}; // Return an empty variant in case of an error
//                 }

//                 if (task_param.hasMember("group")) {
//                     cartesian_goal.planning_group = static_cast<std::string>(task_param["group"]);
//                 }
//                 else {
//                     ROS_ERROR("No group specified!");
//                     return std::monostate{}; // Return an empty variant in case of an error
//                 }

//                 if (task_param.hasMember("merge") && task_param["merge"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
//                     if(static_cast<bool>(task_param["merge"])){
//                         if (last_planned_configuration_) {
//                             std::cout << (*last_planned_configuration_).first << std::endl;
//                             std::cout << cartesian_goal.planning_group << std::endl;

//                             if((*last_planned_configuration_).first==cartesian_goal.planning_group){
//                                 cartesian_goal.initial_configuration = (*last_planned_configuration_).second; // Use the shared vector
//                             } else {
//                                 waitForExecutions();
//                             }
//                         } else {
//                             ROS_ERROR("No previous configuration available for merge.");
//                             return std::monostate{}; // Return an empty variant in case of an error
//                         }
//                     }
//                     else {
//                         waitForExecutions();
//                     }
//                 }

//                 return cartesian_goal;
//             } else if (task_type == "JointPlan") {
//                 planning_msgs::JointPlanGoal joint_goal;

//                 if (task_param.hasMember("goal")) {
//                     XmlRpc::XmlRpcValue goal = task_param["goal"];
//                     for (int i = 0; i < goal.size(); ++i) {
//                         if (goal[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
//                             double value = static_cast<double>(goal[i]);
//                             joint_goal.goal_configuration.push_back(value);
//                         }
//                     }
//                 }
//                 else {
//                     ROS_ERROR("No goal specified!");
//                     return std::monostate{}; // Return an empty variant in case of an error
//                 }

//                 if (task_param.hasMember("group")) {
//                     joint_goal.planning_group = static_cast<std::string>(task_param["group"]);
//                 }
//                 else {
//                     ROS_ERROR("No group specified!");
//                     return std::monostate{}; // Return an empty variant in case of an error
//                 }

//                 if (task_param.hasMember("merge") && task_param["merge"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
//                     if(static_cast<bool>(task_param["merge"])){
//                         if (last_planned_configuration_) {
//                             if((*last_planned_configuration_).first==joint_goal.planning_group){
//                                 joint_goal.initial_configuration = (*last_planned_configuration_).second; // Use the shared vector
//                             } else {
//                                 waitForExecutions();
//                             }
//                         } else {
//                             ROS_ERROR("No previous configuration available for merge.");
//                             return std::monostate{}; // Return an empty variant in case of an error
//                         }
//                     }
//                     else {
//                         waitForExecutions();
//                     }
//                 }

//                 return joint_goal;
//             }
//         }

//         ROS_ERROR("Wrong task parameters!");
//         return std::monostate{}; // Return an empty variant in case of an error
//     }

//     /**
//      * @brief Execute a task based on the provided goal.
//      *
//      * @param goal Task parameters as a variant type.
//      */
//     bool plan(const PlanningGoal& goal) {
//         if (std::holds_alternative<std::monostate>(goal)) {            
//             ROS_ERROR("Planning goal is invalid.");
//             ros::shutdown();
//             return false;
//         }
//         if (std::holds_alternative<planning_msgs::CartesianPlanGoal>(goal)) {
//             planning_msgs::CartesianPlanGoal cartesian_goal = std::get<planning_msgs::CartesianPlanGoal>(goal);
//             cartesian_client_->sendGoal(cartesian_goal);
//             cartesian_client_->waitForResult();

//             if (cartesian_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//                 ROS_INFO("Cartesian plan succeeded.");
//                 planning_msgs::CartesianPlanResultConstPtr plan_result = cartesian_client_->getResult();
//                 trajectory_msgs::JointTrajectory planned_joint_traj = plan_result->planned_trajectory.joint_trajectory;
//                 std::pair<std::string, std::vector<double>> last_planned_configuration;
//                 last_planned_configuration.first = cartesian_goal.planning_group;
//                 last_planned_configuration.second = planned_joint_traj.points.back().positions;
//                 last_planned_configuration_ = std::make_unique<std::pair<std::string, std::vector<double>>>(last_planned_configuration);  
//                 planning_msgs::ExecutePlanGoal execute_goal;
//                 execute_goal.motion_plan = plan_result->planned_trajectory;
//                 execute_goal.move_group_name = cartesian_goal.planning_group;
//                 execute_goal_queue.push(execute_goal);
//             } else {
//                 ROS_ERROR("Cartesian plan failed.");
//                 return false;
//             }
//         } else if (std::holds_alternative<planning_msgs::JointPlanGoal>(goal)) {
//             planning_msgs::JointPlanGoal joint_goal = std::get<planning_msgs::JointPlanGoal>(goal);
//             joint_client_->sendGoal(joint_goal);
//             joint_client_->waitForResult();

//             if (joint_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//                 ROS_INFO("Joint plan succeeded.");
//                 planning_msgs::JointPlanResultConstPtr plan_result = joint_client_->getResult();
//                 trajectory_msgs::JointTrajectory planned_joint_traj = plan_result->planned_trajectory.joint_trajectory;
//                 std::pair<std::string, std::vector<double>> last_planned_configuration;
//                 last_planned_configuration.first = joint_goal.planning_group;
//                 last_planned_configuration.second = planned_joint_traj.points.back().positions;
//                 last_planned_configuration_ = std::make_unique<std::pair<std::string, std::vector<double>>>(last_planned_configuration);  
//                 planning_msgs::ExecutePlanGoal execute_goal;
//                 execute_goal.motion_plan = plan_result->planned_trajectory;
//                 execute_goal.move_group_name = joint_goal.planning_group;
//                 execute_goal_queue.push(execute_goal);
//             } else {
//                 ROS_ERROR("Joint plan failed.");
//                 return false;
//             }
//         } else {
//             ROS_ERROR("Unexpected Plan TYPE!");
//             return false;
//         }

//         return true;
//     }

//     void waitForExecutions() {
//         while(!isExecutionQueueEmpty() && isExecutionThreadRunning()){
//             ros::spinOnce();  // Allow ROS to process callbacks
//             ros::Duration(0.01).sleep();  // Sleep for a while
//         }
//     } 
//     bool isExecutionQueueEmpty(){
//         return execute_goal_queue.empty();
//     }
//     bool isExecutionThreadRunning(){
//         return run_thread;
//     }
//     void stopExecutionThread() {
//         run_thread = false;
//         if (execution_thread_.joinable()) {
//             execution_thread_.join();
//         }
//     }   

// private:
//     ros::NodeHandle& nh_;
//     bool is_executing_;
//     bool run_thread = true;
//     std::queue<planning_msgs::ExecutePlanGoal> execute_goal_queue;
//     std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::CartesianPlanAction>> cartesian_client_;
//     std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::JointPlanAction>> joint_client_;
//     std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>> execute_client_;
//     std::unique_ptr<std::pair<std::string,std::vector<double>>> last_planned_configuration_; // Use a shared pointer with a vector
//     std::thread execution_thread_;

//     /**
//      * @brief Handle task execution in a separate thread.
//      */
//     void handleExecution() {
//         while (ros::ok() && run_thread) {
//             if (!execute_goal_queue.empty()) {
//                 if (is_executing_) {
//                     execute_client_->waitForResult();
//                     if (execute_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//                         execute_goal_queue.pop();
//                         if(execute_goal_queue.empty()) {
//                             is_executing_ = false;
//                             continue;
//                         }
//                         planning_msgs::ExecutePlanGoal execute_goal = execute_goal_queue.front();
//                         execute_client_->sendGoal(execute_goal);
//                     } else {
//                         execute_goal_queue.pop();
//                         ROS_ERROR("Execution failed.");
//                         run_thread = false;
//                     }
//                 } else {
//                     std::cout << "Executing Trajectory" << std::endl;
//                     planning_msgs::ExecutePlanGoal execute_goal = execute_goal_queue.front();
//                     execute_client_->sendGoal(execute_goal);
//                     is_executing_ = true;
//                 }
//             } else {
//                 // Sleep for a while if the queue is empty to avoid busy-waiting
//                 std::this_thread::sleep_for(std::chrono::milliseconds(100));
//             }
//         }
//         std::cout << "Killing the thread" << std::endl;
//     }
// };


#include <ros/ros.h>
#include <XmlRpcValue.h>
#include "planning_pkg/actions.h"
#include "planning_pkg/task_handler.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_planner_node");
    ros::NodeHandle nh;

    TaskHandler task_handler(nh);

    XmlRpc::XmlRpcValue task_list;
    if (nh.getParam("/tasks", task_list)) {
        if (task_list.size() > 0) {
            for (int i = 0; i < task_list.size(); ++i) {
                XmlRpc::XmlRpcValue task = task_list[i];
                PlanningGoal planning_goal = task_handler.convertYamlToGoal(task);
                if(!task_handler.plan(planning_goal)) break;
            }
        } else {
            ROS_ERROR("Task List is Empty!");
            return 1;
        }
    } else {
        ROS_ERROR("Failed to retrieve the /task_list parameter from the parameter server.");
        return 1;
    }
    while(!task_handler.isExecutionQueueEmpty() && task_handler.isExecutionThreadRunning()){
        ros::spinOnce();  // Allow ROS to process callbacks
        ros::Duration(0.1).sleep();  // Sleep for a while
    }
    task_handler.stopExecutionThread();
    nh.shutdown();
    return 0;
}
