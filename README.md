# Multi-Robot, Energy Efficient Exploration

All algorithms used for this project are outlines in `Robotics_Final_Project_Report.pdf`

## Dependencies

## User Guide

```bash
$ roslaunch launchers multi-sim-mapping # move base, gmapping, two robots and rviz
$ rosrun explorer explorer_server_node.py # shows the frontiers, but does not of the functionality
$ rosrun explorer explorer_client_node.py # Runs the clients with the server to go out and explore the map
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
