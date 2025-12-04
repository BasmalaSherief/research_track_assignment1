# Research Track I - Assignment 1

## Description
This package (`assignment1_rt`) implements a control system for the Turtlesim simulation using ROS 2 (C++). It satisfies the requirements for Assignment 1 of the Research Track I course.

The project consists of two main nodes:
1.  **UI Node:** A controller that allows the user to spawn commands to a specific turtle. It handles linear and angular velocity and implements a 1-second movement timer.
2.  **Distance Monitor Node:** A safety node that monitors the position of both turtles. It calculates the Euclidean distance and stops the robots if they get too close or if they approach the environment boundaries.

## Installation and Build
To run this package, you must first build it in your ROS 2 workspace.

```bash
cd ~/ros2_ws
colcon build --packages-select assignment1_rt
source install/setup.bash

How to Run

You can run the assignment using the provided shell script (easier) or manually via separate terminals.
Option 1: Using the Shell Script (Recommended)

A run_assignment.sh script is provided to automate the startup process. It launches the simulation, spawns the second turtle, starts the distance monitor but you run the user interface from another terminal

    Make the script executable:
    Bash

chmod +x run_assignment.sh

Run the script:
Bash

    ./run_assignment.sh

Option 2: Manual Execution

If you prefer to run the nodes individually, open four separate terminals and run the following commands:

Terminal 1: Start Simulation
Bash

ros2 run turtlesim turtlesim_node

Terminal 2: Spawn Second Turtle We run the script directly from the source folder:
Bash

source install/setup.bash
python3 src/assignment1_rt/src/turtle_spawn.py

Terminal 3: Run Distance Monitor
Bash

source install/setup.bash
ros2 run assignment1_rt distance_node

Terminal 4: Run UI Controller
Bash

source install/setup.bash
ros2 run assignment1_rt UI

Project Structure

    src/UI.cpp: C++ node that handles user input (std::cin) and publishes cmd_vel commands. It includes logic to stop the robot automatically after 1 second.

    src/distance_node.cpp: C++ node that subscribes to /turtle1/pose and /turtle2/pose. It publishes the relative distance and sends stop commands if boundaries are breached or collision is imminent.

    src/turtle_spawn.py: Python script that uses the /spawn service to add turtle2 to the environment.

    run_assignment.sh: Bash script to automate the launch of all nodes and the UI terminal.

    launch/simulation.launch.py: (Optional) ROS 2 Launch file configuration.

