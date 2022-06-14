#!/usr/bin/env python
from asyncio import wait_for
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
        self.done = 0
        self.rotate = True

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('converted_message', String, self.callback_fn)

        moveit_commander.roscpp_initialize(sys.argv) 
        self.robot = moveit_commander.RobotCommander() 
        self.scene = moveit_commander.PlanningSceneInterface() 
        self.gripper = moveit_commander.MoveGroupCommander('gripper')
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.arm.set_planning_time(2) # do we need this only for arm?
        self.gripper.set_planning_time(2)
    
    def move_to_position(self, goal):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        client.send_goal(goal)
        wait = wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
			return False
    		else:
        		return True

                
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

    def stop_robot(self):
        self.moving = False
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
        self.stop+=1

        # task is done
        


    def move(self,d):
        self.moving = True

        twist = Twist()
        if d == 'f':
            twist.linear.x = 0.1; twist.linear.y = 0.0; twist.linear.z = 0.0
        elif d == 'b':
            twist.linear.x = -0.1; twist.linear.y = 0.0; twist.linear.z = 0.0
        else:
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.x = 0
            self.stop+=1
        
        self.pub.publish(twist)

    def fix_target(self, m, e):
        twist = Twist()
        if m > IMAGE_SIZE_X / 2 + e:
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = -0.2
        elif m < IMAGE_SIZE_X / 2 - e:
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.2
        else:
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

        self.pub.publish(twist)
    

    # TODO something is not correct here :cry
    def callback_fn(self, s):
        if self.done == True:
            return

        m = s.data.split()
        if m[0] == 'bottle' or m[0] == 'vase': # msg converter sends only this msgs but anyway check it 
            # find middle of bottle's bounding box
            mid = (int(m[2]) + int(m[1]))/2

            e = 30
            if mid > IMAGE_SIZE_X / 2 + e or mid < IMAGE_SIZE_X / 2 - e:
                # if the center of bounding box is not at screen center with some error range, rotate
                self.fix_target(mid, e)

            # move toward bottle until the size of its bounding box >= critical size
            if int(m[2]) - int(m[1]) < CRITICAL_BOX_SIZE:
                self.move('f')
            else:
                self.stop_robot()
                self.pick()

            if self.stop == 2:
                twist = Twist()
                self.pub.publish(twist)
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                self.pub.publish(twist)

                rospy.sleep(0.5)
                self.pick()
                self.stop += 1
                rospy.sleep(15)
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = 1
                goal.target_pose.pose.position.y = 0
                goal.target_pose.pose.position.z = 0
                goal.target_pose.pose.orientation.x = 0
                goal.target_pose.pose.orientation.y = 0
                goal.target_pose.pose.orientation.z = 1
                goal.target_pose.pose.orientation.w = 0

                res = self.move_to_position(goal)
                if res:
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = 0.2
                    goal.target_pose.pose.position.y = -0.2
                    goal.target_pose.pose.position.z = 0
                    goal.target_pose.pose.orientation.x = 0
                    goal.target_pose.pose.orientation.y = 0
                    goal.target_pose.pose.orientation.z = 1
                    goal.target_pose.pose.orientation.w = 0
                    res = self.move_to_position(goal)
                    if res:
                        self.pick()


def main():
    # let's see if the order changed, what will happen to error messages 
    rospy.init_node('task1', anonymous=True)
    task1()

    rospy.spin()


if __name__=="__main__":
    main()
