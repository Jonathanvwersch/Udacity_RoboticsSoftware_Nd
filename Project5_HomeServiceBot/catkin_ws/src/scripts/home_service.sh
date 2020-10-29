#!/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$catkin_ws/src/world/playground.world" &
sleep 10

xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch world_file:=$catkin_ws/src/map/playground.yaml" &
sleep 10

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 20

xterm -e "rosrun pick_objects pick_objects" &

sleep 5

xterm -e "rosrun add_markers add_markers" 
