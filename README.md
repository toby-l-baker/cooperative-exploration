# cooperative-exploration
Energy Efficient, Collaborative Robot Exploration

## To-Do

1. Implement ExplorerServer and ExplorerClient to communicate
   1. ExplorerClient indicates it needs a target
   2. ExplorerServer uses its known list of frontiers and robot location to decide waypoint
   3. ExplorerClient takes job and travels to waypoint
2. Integrate map merging package
   1. Each robot's gmapping system should create their own map (/robot0/map, /robot0/map\_metadata)
   2. The multirobot\_map\_merge package should combine the maps and publish a uniform map to /map
   3. Each robot's move\_base node can listen to the global /map topic to understand where they are
3. Implement a node to measure what % of the map has been mapped over time

### Tarkan

#### Accomplished

1. Created packages (map for maps, launchers for launch files, nodes for general nodes)
2. Created a basic map that captures some of our simplest use cases
1. Add a robot with sensor to stdr sim in the launch file in launchers
1. Add a launch file that starts teleop control for a given robot
1. Created a node to measure current path length of a robot
1. Started explorer package
   1. ExploreServer and ExplorerClient class shells
   1. MapListener class shell to listen to and parse maps
1. Integrated move\_base with STDR sim

#### Planned

1. Create Service files for ExplorerServer and ExplorerClient to communicate
2. Implement MapListener to listen to global map topic and find frontiers

### Toby 

### Accomplished
1. Initialise repo
2. Setup gmapping with STDR Sim
3. Reading: Energy Efficient Exploration for Mobile Robots, Fronteir Exploration, Frontier Exploration for Mobile Robots.
3. Setup frontier exploration package
1. Spawn multiple robots
2. Have multiple robots mapping
1. Integrate multiple map merging package (use explorer as a guide)
1. Setup Frontier Location Framwork
1. Frontier ID debugging
1. Juggle around map listener interface so it is portable
1. Create Explorer Server Functionality

### Planned



## Install Instructions

1. Install ROS Distro for your OS
2. Install [STDR-Simulator ROS package](http://wiki.ros.org/stdr_simulator/Tutorials/Set%20up%20STDR%20Simulator): I had to install from git as I am using Melodic.
1. Install [turltebot\_teleop ROS package](http://wiki.ros.org/turtlebot_teleop) for teleop control
2. Install [explore\_lite](http://mirror-eu.wiki.ros.org/explore_lite.html?distro=kinetic)
2. Install [move\_base](http://wiki.ros.org/move_base)
2. Install [gmapping](http://wiki.ros.org/gmapping)

## Launching our explorers

```bash
$ roslaunch launchers multi-sim-mapping # move base, gmapping, two robots and rviz
$ rosrun explorer explorer_server_node.py # shows the frontiers, but does not of the functionality
$ rosrun explorer explorer_client_node.py # Runs the clients with the server to go out and explore the map
```
