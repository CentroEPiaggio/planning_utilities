Initial version of the planning_utilities for controlling robotic manipulators at Research Center "Enrico Piaggio" using MoveIt.

# planning_utilities - ROS Metapackage

The `planning_utilities` metapackage is a comprehensive suite of tools and ROS nodes designed to streamline robotic path planning and execution tasks. It offers various planning actions and provides a convenient way to manage and execute tasks involving Cartesian planning, joint planning, and plan execution.

## Overview

In the field of robotics, path planning and execution are essential for enabling robots to perform tasks accurately and safely. The `planning_utilities` metapackage simplifies these tasks by providing a collection of tools and ROS nodes that facilitate the entire process.

## Packages

### 1. `planning_pkg`

The `planning_pkg` package, a crucial component of the `planning_utilities` metapackage, contains several ROS action servers and task execution functionalities. Here are the key functionalities offered by the `planning_pkg` package:

#### a. Action Servers
   - **`CartesianPlanActionServer`**: This class serves as a base for a Cartesian Plan Action Server. It handles Cartesian path planning goals, manages trajectories, and executes them for specified planning groups.

   - **`JointPlanActionServer`**: Similar to `CartesianPlanActionServer`, this class serves as a base for a Joint Plan Action Server. It handles joint path planning goals, manages joint configurations, and executes them for specified planning groups.

   - **`ExecutePlanActionServer`**: This class enables the execution of planned trajectories using an action server. It manages the execution of motion plans and provides callback functions for goal completion.

#### b. Task Handler
   - The `TaskHandler` class is designed to handle task execution using ROS action clients. It provides functionalities to plan and execute tasks based on specified goals. Tasks can involve both Cartesian and joint planning goals.

### 2. `planning_msgs`

The `planning_msgs` package defines custom ROS action and message types used by the action servers and other components in the `planning_utilities` metapackage. These message types facilitate the communication of planning goals and results between nodes.

## Usage

To use the `planning_utilities` metapackage in your ROS environment, follow these steps:

1. **Clone the Package**: Clone the `planning_utilities` package to your ROS workspace.

2. **Build the Package**: Use `catkin_make` to build the package within your workspace.

3. **Launch the Nodes**: Use the provided launch files to start the ROS nodes. For example, you can use the `joint_plan_client.launch` file to launch the client nodes. This launch file loads task parameters from a YAML file and launches the client nodes responsible for task planning and execution.

4. **Define Tasks**: Define tasks and goals in YAML format, as shown in the example below. These tasks can include Cartesian and joint planning goals and options for merging or not merging planned configurations.