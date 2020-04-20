# cooperative-exploration
Energy Efficient, Collaborative Robot Exploration

## To-Do

### Tarkan

#### Accomplished

1. Created map package
1. Created launchers package
2. Created a basic map that captures some of our simplest use cases
1. Add a robot to stdr sim in the launch file in launchers
2. Add a sensor to that robot in the launch file in launchers
1. Add a launch file that starts teleop control for a given robot
1. Created a node to measure current path length of a robot
1. Added a very basic explorer package that provides a place for explorer code to go

#### Planned

1. Create a mapping node
    1. Subscribe to laser data
    2. Initialize an occupancy grid
    3. Iteratively update occupancy grid with sensor data
    1. Visualize occupancy grid in RVIZ
    1. Create launch file for occupancy grid
    1. Expose occupancy grid as a useful topic
1. Try out existing exploration packages
1. Try out Move Base on the simulation, maybe in conjunction with above


### Toby 

### Accomplished
1. Initialise repo
2. Setup gmapping with STDR Sim
3. Reading: Energy Efficient Exploration for Mobile Robots, Fronteir Exploration, Frontier Exploration for Mobile Robots.

### Planned
1. Spawn multiple robots
2. Have multiple robots mapping

## Install Instructions

1. Install ROS Distro for your OS
2. Install [STDR-Simulator ROS package](http://wiki.ros.org/stdr_simulator/Tutorials/Set%20up%20STDR%20Simulator): I had to install from git as I am using Melodic.
1. Install [turltebot\_teleop ROS package](http://wiki.ros.org/turtlebot_teleop) for teleop control
