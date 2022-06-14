###Rasperry_PI
```sh
roslaunch turtlebot3_bringup turtlebot3_robot.launch	
```
	
###Jetson	
```sh
roscore
roslaunch ros_deep_learning video_source.ros1.launch input:=csi://0 output:=display://0
python converter.py
roslaunch darknet_ros darknet_ros.launch
```        
              
###REMOTE_PC
```sh
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
	
####Move for map
```sh
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/empty_arena_new.yaml open_rviz:=false
rosrun task2.py
```
