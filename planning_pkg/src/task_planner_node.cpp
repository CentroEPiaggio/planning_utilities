#include <ros/ros.h>
#include <XmlRpcValue.h>
#include "planning_pkg/actions.h"
#include <variant>
// Define a custom type for task parameters
using TaskParameters = std::variant<planning_msgs::CartesianPlanGoal, planning_msgs::JointPlanGoal>;

// Function to parse a Pose from XML-RPC data
geometry_msgs::Pose parsePose(const XmlRpc::XmlRpcValue& pose_param)
{
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

// Function to convert YAML data to goal message
TaskParameters convertYamlToGoal(const XmlRpc::XmlRpcValue& task_param)
{
    if (task_param.hasMember("type"))
    {
        std::string task_type = static_cast<std::string>(task_param["type"]);

        if (task_type == "CartesianPlan")
        {
            planning_msgs::CartesianPlanGoal cartesian_goal;

            if (task_param.hasMember("goal"))
            {
                cartesian_goal.goal_pose = parsePose(task_param["goal"]);
            }

            if (task_param.hasMember("initial") && task_param["initial"].getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                for (int i = 0; i < task_param["initial"].size(); ++i)
                {
                    if (task_param["initial"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    {
                        double value = static_cast<double>(task_param["initial"][i]);
                        cartesian_goal.initial_configuration.push_back(value);
                    }
                }
            }

            if (task_param.hasMember("group"))
            {
                cartesian_goal.planning_group= static_cast<std::string>(task_param["group"]);

            }

            // Handle other fields if needed

            return cartesian_goal;
        }
        else if (task_type == "JointPlan")
        {
            planning_msgs::JointPlanGoal joint_goal;

            if (task_param.hasMember("goal"))
            {
                // Extract and assign the goal parameters for JointPlan
                XmlRpc::XmlRpcValue goal = task_param["goal"];
                for (int i = 0; i < goal.size(); ++i)
                {
                    if (goal[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    {
                        double value = static_cast<double>(goal[i]);
                        joint_goal.goal_configuration.push_back(value);
                    }
                }
            }

            if (task_param.hasMember("initial") && task_param["initial"].getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                for (int i = 0; i < task_param["initial"].size(); ++i)
                {
                    if (task_param["initial"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    {
                        double value = static_cast<double>(task_param["initial"][i]);
                        joint_goal.initial_configuration.push_back(value);
                    }
                }
            }

            if (task_param.hasMember("group"))
            {
                joint_goal.planning_group= static_cast<std::string>(task_param["group"]);

            }

            // Handle other fields if needed

            return joint_goal;
        }
    }

    ROS_ERROR("Wrong task parameters!");
    return TaskParameters(); // Return an empty variant in case of an error
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_planner_node");
    ros::NodeHandle nh;

    // Create shared pointers for action clients
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::CartesianPlanAction>> cartesian_client;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::JointPlanAction>> joint_client;
    std::shared_ptr<actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>> execute_client;

    // Wait for action servers to start
    cartesian_client.reset(new actionlib::SimpleActionClient<planning_msgs::CartesianPlanAction>("cartesian_plan_action", true));
    joint_client.reset(new actionlib::SimpleActionClient<planning_msgs::JointPlanAction>("joint_plan_action", true));
    execute_client.reset(new actionlib::SimpleActionClient<planning_msgs::ExecutePlanAction>("execute_plan_action", true));

    if (!cartesian_client->waitForServer(ros::Duration(5.0)) ||
        !joint_client->waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR("One or more action servers did not start within the timeout.");
        return 1;
    }
    bool is_executing = false;
    // Retrieve the task list from the ROS parameter server as XML-RPC data
    XmlRpc::XmlRpcValue task_list;
    if (nh.getParam("/tasks", task_list))
    {
        if (task_list.size()>0)
        {
            // Iterate over tasks and execute them
            for (int i = 0; i < task_list.size(); ++i)
            {
                XmlRpc::XmlRpcValue task = task_list[i];
                TaskParameters goal = convertYamlToGoal(task);
                if (std::holds_alternative<planning_msgs::CartesianPlanGoal>(goal))
                {
                    planning_msgs::CartesianPlanGoal cartesian_goal = std::get<planning_msgs::CartesianPlanGoal>(goal);
                    cartesian_client->sendGoal(cartesian_goal);

                    cartesian_client->waitForResult();

                    if (cartesian_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        ROS_INFO("Cartesian plan succeeded.");
                        
                        if(is_executing) 
                        {
                            execute_client->waitForResult();
                            if (execute_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
                            {
                                // Set new goal
                                execute_client::ExecutePlanGoal execute_goal;
                                execute_goal.move_group_name = cartesian_goal.planning_group; 
                                execute_goal.motion_plan = cartesian_client->getResult()->planned_trajectory;
                                execute_client->sendGoal(execute_goal);
                            }
                            else 
                            {
                                // RAISE EXCEPTION
                                ROS_ERROR("Execution FAILED!");
                                return 1;
                            }
                        }
                        else
                        {
                            is_executing = true;
                            // Set new goal
                            execute_client::ExecutePlanGoal execute_goal;
                            execute_goal.move_group_name = cartesian_goal.planning_group; 
                            execute_goal.motion_plan = cartesian_client->getResult()->planned_trajectory;
                            execute_client->sendGoal(execute_goal);
                        }
                        
                    }
                    else
                    {
                        ROS_ERROR("Cartesian plan failed.");
                        return 1;
                    }
                }
                else if (std::holds_alternative<planning_msgs::JointPlanGoal>(goal))
                {
                    planning_msgs::JointPlanGoal joint_goal = std::get<planning_msgs::JointPlanGoal>(goal);
                    joint_client->sendGoal(joint_goal);

                    joint_client->waitForResult();

                    if (joint_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        ROS_INFO("Joint plan succeeded.");
                        
                        if(is_executing) 
                        {
                            execute_client->waitForResult();
                            if (execute_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
                            {
                                execute_client::ExecutePlanGoal execute_goal;
                                execute_goal.move_group_name = joint_goal.planning_group; 
                                execute_goal.motion_plan = joint_client->getResult()->planned_trajectory;
                                execute_client->sendGoal(execute_goal);                            
                            }
                            else 
                            {
                                // RAISE EXCEPTION
                            }
                        }
                        else
                        {
                            is_executing = true;
                            // Set new goal
                            execute_client::ExecutePlanGoal execute_goal;
                            execute_goal.move_group_name = joint_goal.planning_group; 
                            execute_goal.motion_plan = joint_client->getResult()->planned_trajectory;
                            execute_client->sendGoal(execute_goal);                          
                        }
                        
                    }
                    else
                    {
                        ROS_ERROR("Joint plan failed.");
                        return 1;
                    }
                }
            }
        }
        else 
        {
        ROS_ERROR("Task List is Empty!");
        return 1;
        }
    }
    else
    {
        ROS_ERROR("Failed to retrieve the /task_list parameter from the parameter server.");
        return 1;
    }
    return 0;
}
