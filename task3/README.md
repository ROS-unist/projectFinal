Turtlebot_PI
	roslaunch turtlebot3_bringup turtlebot3_robot.launch	
	
	
Jetson	
	roscore

	roslaunch ros_deep_learning video_source.ros1.launch input:=csi://0 output:=display://0

	python converter.py

        roslaunch darknet_ros darknet_ros.launch
        
        
        
        
        
REMOTE_PC
	roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
	roslaunch turtlebot3_manipulation_moveit_config move_group.launch
	rosrun basic_control teleop
	
Move
	roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/empty_arena_new.yaml open_rviz:=false
	rosrun task3_slam.py
	rosrun task3_yolo.py
	
stop turtlebot movement
	rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
	
	
