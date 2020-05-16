# Multi-Robot, Energy Efficient Exploration

All algorithms used for this project are outlines in `Robotics_Final_Project_Report.pdf`

## Dependencies

1. A ROS Distro, we used ROS Melodic on Ubuntu 18.04 see [https://www.ros.org/]
2. m-explore
```
cd catkin_ws/src/
git clone https://github.com/hrnr/m-explore.git
```
3. stdr-simulator
```
cd catkin_ws/src/
git clone https://github.com/stdr-simulator-ros-pkg/stdr_simulator.git
```
3. `sudo apt-get install ros-<distro>-slam-gmapping ros-<distro>-move-base ros-<distro>-navigation

## User Guide

To launch one robot issue the commands below

```bash
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch launchers demo_one_robot.launch 
```

To launch two robots use the commands below

```bash
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch launchers demo_two_robots.launch 
```

## Project Structure

Only important files covered here.

```
catkin_ws/                       # main ROS workspace
   src/
      explorer/                  # source files for our implementation
         src/                    
            front_test/          # code for testing algorithms and plotting occupancy grids with matplotlib
            pyexplorer/          # source code for client, server, frontier search and map_listener
            explorer_client_node.py    # ROS node for a client, instantiated once for each robot
            explorer_server_node.py    # ROS node for server, instantiated once on startup
         srv/                    # service types for server requests/responses
      
      launchers/                 # launch file package
         resources/              # move_base config, robot config, rviz configs
         launch/                 # all launch files
            demo_one_robot.launch      # run our implementation with one robot
            demo_two_robot.launch      # run our implementation with two robots
            demo_three_robots.launch   # run our implementation with three robots
            multi_sim_exploring.launch # launch two robots and use explore-lite to explore
```
