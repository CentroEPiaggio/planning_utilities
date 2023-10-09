#include <ros/ros.h>
#include "planning_pkg/actions.h"

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "motion_planning_server_node");
    ros::NodeHandle nh;

    // Create instances of your action server classes
    CartesianPlanActionServer cartesian_server(nh, "cartesian_plan_action");
    JointPlanActionServer joint_server(nh, "joint_plan_action");
    // Spin to keep the node alive
    ros::spin();

    return 0;
}
