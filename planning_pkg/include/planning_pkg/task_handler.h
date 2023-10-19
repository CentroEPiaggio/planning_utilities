#ifndef TASK_HANDLER_H
#define TASK_HANDLER_H

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include "planning_pkg/actions.h"
#include <variant>
#include <queue>
#include <actionlib/client/simple_action_client.h>
#include <thread>
#include <chrono>

// Define a variant type for task parameters
 using PlanningGoal = std::variant<std::monostate, planning_msgs::CartesianPlanGoal, planning_msgs::JointPlanGoal>;

/**
 * @brief TaskHandler class for handling task execution using ROS action clients.
 *
 * TaskHandler provides methods to plan and execute tasks using ROS action clients.
 * It can handle both Cartesian and Joint planning goals.
 *
 * @author Alessandro Palleschi
 */
class TaskHandler {
public:
    /**
     * @brief Constructor for TaskHandler.
     *
     * @param nh Reference to the ROS NodeHandle.
     */
    TaskHandler(ros::NodeHandle& nh);

    /**
     * @brief Destructor for TaskHandler, joins the execution thread.
     */
    ~TaskHandler();


    /**
     * @brief Convert YAML data to a task parameter variant.
     *
     * @param task_param XML-RPC data containing task parameters.
     * @return PlanningGoal variant representing the task goal.
     */
    PlanningGoal convertYamlToGoal(const XmlRpc::XmlRpcValue& task_param);

    /**
     * @brief Plan and execute a task based on the provided goal.
     *
     * @param goal Task parameters as a variant type.
     * @return True if the task planning and execution were successful, false otherwise.
     */
    bool plan(const PlanningGoal& goal);

    /**
     * @brief Stop the execution thread gracefully.
     */
    void stopExecutionThread();

    /**
     * @brief Check if the execution queue is empty.
     *
     * @return True if the execution queue is empty, false otherwise.
     */
    bool isExecutionQueueEmpty();
    
    /**
     * @brief Parse a Pose from XML-RPC data.
     *
     * @param pose_param XML-RPC data containing pose information.
     * @return Parsed geometry_msgs::Pose.
     */
    geometry_msgs::Pose parsePose(const XmlRpc::XmlRpcValue& pose_param);

    /**
     * @brief Check if the execution thread is running.
     *
     * @return True if the execution thread is running, false otherwise.
     */
    bool isExecutionThreadRunning();

    void waitForExecutions();

private:
    ros::NodeHandle& nh_;
    bool is_executing_;
    bool run_thread = true;
    std::queue<planning_msgs::ExecutePlanGoal> execute_goal_queue;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::CartesianPlanAction>> cartesian_client_;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::JointPlanAction>> joint_client_;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>> execute_client_;
    std::unique_ptr<std::pair<std::string, std::vector<double>>> last_planned_configuration_;
    std::thread execution_thread_;

    /**
     * @brief Handle task execution in a separate thread.
     *
     * This function continuously checks for tasks in the execution queue and
     * executes them using ROS action clients.
     */
    void handleExecution();

    // Other member functions and private data members...
};

#endif // TASK_HANDLER_H
