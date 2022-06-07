#!/usr/bin/env python
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import moveit_commander
import moveit_msgs.msg
import rospy

import sys

IMAGE_SIZE_X = 1280
CRITICAL_BOX_SIZE = 1280 / 10

class task1:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('converted_message', String, self.callback_fn)
        self.pub2 = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        moveit_commander.roscpp_initialize(sys.argv) 
        self.robot = moveit_commander.RobotCommander() 
        self.scene = moveit_commander.PlanningSceneInterface() 
        self.gripper = moveit_commander.MoveGroupCommander('gripper')
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.arm.set_planning_time(2)

        # initial postion of gripper
        self.move_arm([0,0,0,0])


    def move_gripper(self, v):
        joint_gripper = self.gripper.get_current_joint_values() # is this necessary??
        joint_gripper = v
        self.gripper.go(joints=joint_gripper, wait=True)
        rospy.sleep(5)
        self.gripper.stop()
        self.gripper.clear_pose_targets()

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        self.pub2.publish(display_trajectory)

    def move_arm(self, v):
        joint_values = self.arm.get_current_joint_values() 
        joint_values = v

        self.arm.go(joints=joint_values, wait=True)
        rospy.sleep(5)
        self.arm.stop()
        self.arm.clear_pose_targets()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.pub.publish(twist)

    def pick(self):
        # open gripper
        self.move_gripper(0.010)

        # arm forward
        self.move_arm([0,0,0,0])

        # close gripper 
        self.move_gripper(-0.010)

        # arm home pose
        self.move_arm([0.0, -1.0, 0.3, 0.7])

        # stop robot completely by passing random char to move
        self.move('s')

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
        
        self.pub.publish(twist)

    def fix_target(self, m, e = 50):  # higher accuracy
        twist = Twist()
        if m > IMAGE_SIZE_X / 2 + e:
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = -0.2
        elif m < IMAGE_SIZE_X / 2 - e:
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.2
        else:
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

        self.pub.publish(twist)
    

    # TODO receive array of Strings not just one
    def callback_fn(self, s):
        m = s.data.split()
        if m[0] == 'bottle':
            # find middle of bottle's bounding box
            mid = (int(m[2]) + int(m[1]))/2

            # if the center of bounding box is not at screen center with some error range, rotate
            self.fix_target(mid)

            # move toward bottle until the size of its bounding box >= critical size
            if int(m[2]) - int(m[1]) < CRITICAL_BOX_SIZE:
                self.move('f')
                rospy.sleep(5)
            else:
                self.pick()


def main():
    c = task1()
    rospy.init_node('task1', anonymous=True)
    rospy.spin()


if __name__=="__main__":
    main()
    