# PA3, CS 81
By Garret Andreine
11/1/2020

## How To Execute
- First, make sure to start the ROS enviroment using the ```roscore``` command  
- Second, Open up the ROS world map with ```rosrun map_server map_server /opt/ros/kinetichare/turtlebot_stage/maps/maze.yaml``` commands
- Third, type ```rviz```
- Fourth, type ```python Q1.py``` 
- In RViz make sure to import the map, the marker, and the axis to get desired results
- Upon executing the Python program you will be asked for start/goal coordinates. These must be entered in the format of x,y


## Requirements
- Use Python 3.3 when running the python scripts
- Make sure to have the ROS enviroment installed
- Make sure to have standard python libraries (such as math) installed
- Each command specified in "How To Execute" should be run in a seperate command line window