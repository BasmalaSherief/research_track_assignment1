#!/bin/bash

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting Assignment 1..."

# Turtlesim 
ros2 run turtlesim turtlesim_node &
TURTLE_PID=$!
sleep 2  

# Python Spawner script
echo "Spawning Turtle 2..."
python3 /home/basmala/ros2_ws/assignment1_rt/src/turtle_spawn.py

# Distance Monitor
echo "Starting Distance Monitor..."
ros2 run assignment1_rt distance_node &
DISTANCE_PID=$!

# Wait loop 
trap "kill $TURTLE_PID $DISTANCE_PID; exit" INT
wait