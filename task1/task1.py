#!/usr/bin/env python
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
import moveit_commander
import moveit_msgs.msg
import rospy

import sys

IMAGE_SIZE_X = 1280
CRITICAL_BOX_SIZE = 1280 / 10

class task1:
    def __init__(self):
        self.pub1 = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.callback_fn)
        self.pub2 = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        moveit_commander.roscpp_initialize(sys.argv) # initialize moveit commander
        self.robot = moveit_commander.RobotCommander() # Instantiate a RobotCommander object
        self.scene = moveit_commander.PlanningSceneInterface() # remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world
        self.gripper = moveit_commander.MoveGroupCommander('gripper') # name of movegroup 2 is “gripper
        self.arm = moveit_commander.MoveGroupCommander('arm') # name of movegroup 1 is “arm”
        self.arm.set_planning_time(2)

    def pick(self):

        '''
        
        related ros topics:
        /joint_trajectory_point
        /gripper_position

        EITHER use the above topics or moveit

        '''
        # open gripper
        joint_gripper = self.gripper.get_current_joint_values()
        joint_gripper[0] = 0.010
        self.gripper.go(joints=joint_gripper, wait=True)
        rospy.sleep(5)
        self.gripper.stop()
        self.gripper.clear_pose_targets()

        # arm forward
        joint_values = self.arm.get_current_joint_values() #How to get joint states
        joint_values[0]= 0
        joint_values[2]= 0
        joint_values[3]= 0
        joint_values[4]= 0

        self.arm.go(joints=joint_values, wait=True)
        rospy.sleep(5)
        self.arm.stop()
        self.arm.clear_pose_targets()

        # close gripper 
        joint_gripper = self.gripper.get_current_joint_values()
        joint_gripper[0] = -0.010
        self.gripper.go(joints=joint_gripper, wait=True)
        rospy.sleep(5)
        self.gripper.stop()
        self.gripper.clear_pose_targets()

        # arm home pose
        joint_values = self.arm.get_current_joint_values() #How to get joint states
        joint_values[0]= 0.0
        joint_values[2]= -1.0
        joint_values[3]= 0.3
        joint_values[4]= 0.7

        self.arm.go(joints=joint_values, wait=True)
        rospy.sleep(5)
        self.arm.stop()
        self.arm.clear_pose_targets()


    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.pub1.publish(twist)
        

    def move(self,d):
        twist = Twist()
        
        if d == 'f':
            twist.linear.x = 0.2; twist.linear.y = 0.0; twist.linear.z = 0.0
        elif d == 'b':
            twist.linear.x = -0.2; twist.linear.y = 0.0; twist.linear.z = 0.0
        else:
            print("Error moving robot! \n Unknown direction.")
            self.stop_robot()
            sys.exit(0)
        
        self.pub1.publish(twist)
    
    def callback_fn(self, data):
        for box in data.bounding_boxes:
            if box.Class == 'bottle':
                # find middle of bottle's bounding box
                mid = (box.xmax + box.xmin)/2
                e = 50 # higher accuracy

                twist = Twist()

                # move toward bottle until the size of its bounding box >= critical size
                if box.xmax - box.xmin < CRITICAL_BOX_SIZE:
                    self.move('f')
                else:
                    self.stop_robot()
                    self.pick()

                # if the center of bounding box is not at screen center with some error range, rotate
                if mid + e > IMAGE_SIZE_X / 2 + e:
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.2
                elif mid < IMAGE_SIZE_X / 2 - e:
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = - 0.2
                else:
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

                self.pub1.publish(twist)

def main():
    c = task1()
    rospy.init_node('task1', anonymous=True)
    rospy.spin()


if __name__=="__main__":
    main()

