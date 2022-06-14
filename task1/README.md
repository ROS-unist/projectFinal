## Turtlebot
```console
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```	
	
## Jetson
```console
roscore
roslaunch ros_deep_learning video_source.ros1.launch input:=csi://0 output:=display://0
python converter.py
roslaunch darknet_ros darknet_ros.launch
python message_converter.py
```
        
## Remote-PC
```console
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
python task1.py
```