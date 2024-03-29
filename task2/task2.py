#!/usr/bin/env python
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import moveit_commander
import rospy

import sys

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

IMAGE_SIZE_X = 1280
CRITICAL_BOX_SIZE = 355 #1280 / 10


class task2:
    def __init__(self):
        self.num_bottles = 0
        self.moving = True
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('converted_message', String, self.callback_fn)

        moveit_commander.roscpp_initialize(sys.argv) 
        self.robot = moveit_commander.RobotCommander() 
        self.scene = moveit_commander.PlanningSceneInterface() 
        self.gripper = moveit_commander.MoveGroupCommander('gripper')
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.arm.set_planning_time(2) # do we need this only for arm?
        self.gripper.set_planning_time(2)
        self.stop_robot()
        self.rotate(90)
        
                
    # destructor to stop the robot at the end
    def __del__(self):
        self.stop_robot()
        moveit_commander.roscpp_shutdown()

    def move_gripper(self, v):
        joint_gripper = self.gripper.get_current_joint_values()
        joint_gripper[0] = v
        self.gripper.go(joints=joint_gripper, wait=True)
        # rospy.sleep(5)
        self.gripper.stop()

        # self.arm.clear_pose_targets() # it is supposed to be used with pose planner

    def move_arm(self, v):
        joint_values = self.arm.get_current_joint_values() 
        joint_values[0] = v[0]; joint_values[1] = v[1]; joint_values[2] = v[2]; joint_values[3] = v[3]

        self.arm.go(joints=joint_values, wait=True)
        rospy.sleep(10)
        self.arm.stop()

        # self.arm.clear_pose_targets() # it is supposed to be used with pose planner

    def rotate(self, angle):
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0
        if angle == 90:
            twist.angular.z = -0.25
            self.pub.publish(twist)
            rospy.sleep(3.5)
            twist.angular.z = 0.25
            rospy.sleep(8)
        elif angle == 180:
            twist.angular.z = 0.5
            self.pub.publish(twist)
            rospy.sleep(7)
            twist.angular.z = 0
        elif angle == 135:
            twist.angular.z = - 0.5
            self.pub.publish(twist)
            rospy.sleep(5.2)
            twist.angular.z = 0
        self.pub.publish(twist)

    def stop_robot(self):
        #self.moving = False
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.pub.publish(twist)

    def pick(self):
        # open gripper
        self.move_gripper(0.010)

        # arm forward
        self.move_arm([0.0, 1.02, -0.4, -0.4538])

        # close gripper 
        self.move_gripper(-0.010)

        # arm home pose
        self.move_arm([0.0, -1.0, 0.3, 0.7])
    
    def drop(self):
        # rotate arm to side
        self.move_arm([0, 1.02, -0.4, -0.4538])

        # open gripper
        self.move_gripper(0.010)

        # amr home pose
        self.move_arm([0.0, -1.0, 0.3, 0.7])

        # task is done
        self.num_bottles += 1


    def move(self,d, t):
        # if self.moving == False:
        #     self.time = rospy.get_time()


        twist = Twist()
        if d == 'f':
            twist.linear.x = 0.07; twist.linear.y = 0.0; twist.linear.z = 0.0
            self.pub.publish(twist)
            rospy.sleep(t)
        elif d == 'b':
            twist.linear.x = -0.07; twist.linear.y = 0.0; twist.linear.z = 0.0
        else:
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.x = 0
        
        self.pub.publish(twist)

    def fix_target(self, m, e):
        twist = Twist()
        if m > IMAGE_SIZE_X / 2 + e:
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = -0.15 #try 0.4
        elif m < IMAGE_SIZE_X / 2 - e:
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.15 # try 0.4
        else:
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

        self.pub.publish(twist)


    def callback_fn(self, s):
        m = s.data.split()
        if m[0] == 'bottle' or m[0] == 'vase' or m[0] == 'person':
            # msg converter sends only this msgs but anyway check it 
            # find middle of bottle's bounding box
            mid = (int(m[2]) + int(m[1]))/2
            dif = int(m[2]) - int(m[1])

            e = 30
            if mid > IMAGE_SIZE_X / 2 + e or mid < IMAGE_SIZE_X / 2 - e:
                # if the center of bounding box is not at screen center with some error range, rotate
                self.fix_target(mid, e)

            # move toward bottle until the size of its bounding box >= critical size
            # if dif < CRITICAL_BOX_SIZE:
                # self.move('f')
            twist = Twist ()

            if self.moving == True:
                self.time = rospy.get_time()
                self.moving = False
            if dif < CRITICAL_BOX_SIZE: 
                if dif < 100:
                    twist.linear.x  = 0.15
                elif dif < 200:
                    twist.linear.x = 0.10
                elif dif < 290:
                    twist.linear.x = 0.05
                elif dif > 290 and dif < CRITICAL_BOX_SIZE:
                    twist.linear.x = 0.05

                self.pub.publish(twist)
            else:
                x = rospy.get_time() - self.time
                self.stop_robot()
                self.pick()
                self.rotate(180)
                self.move('f', x)
                rospy.loginfo(x)
                self.stop_robot()
                self.drop()
                self.moving = True
                if self.num_bottles == 1:
                    self.rotate(135) # change target degree based on where is the 2nd bottle,
                    #create new degree in rotate function
                
                if self.num_bottles >=2:
                    self.stop_robot()
                    return

def main():
    # let's see if the order changed, what will happen to error messages 
    rospy.init_node('task2', anonymous=True)
    c = task2()
    rospy.spin()


if __name__=="__main__":
    main()
