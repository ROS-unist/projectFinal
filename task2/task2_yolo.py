#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import moveit_commander
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import sys

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

IMAGE_SIZE_X = 1280
CRITICAL_BOX_SIZE = 355 #1280 / 10

class simple_motion:
	def __init__(self):
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		#self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.search)
        self.sub = rospy.Subscriber('converted_message', String, self.search)
        moveit_commander.roscpp_initialize(sys.argv) 
        self.robot = moveit_commander.RobotCommander() 
        self.scene = moveit_commander.PlanningSceneInterface() 
        self.gripper = moveit_commander.MoveGroupCommander('gripper')
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.arm.set_planning_time(2) # do we need this only for arm?
        self.gripper.set_planning_time(2)
		self.status_pub = rospy.Publisher('status', String, queue_size=1)
		self.stop_sub = rospy.Subscriber('reset_yolo', Int32, self.set_stop)
		self.bottle_pub = rospy.Publisher('bottle_visible', Bool, queue_size=1)
		self.stop = 0
		self.bottle_collected = 0
		self.num_detected = 0
		self.bottle_pub.publish(False)
		

	def set_stop(self, num):
		self.stop = num.data
		rospy.loginfo(self.stop)
		self.bottle_pub.publish(False)
    
    def pick(self):
        # open gripper
        self.move_gripper(0.010)

        # arm forward
        self.move_arm([0.0, 1.02, -0.4, -0.4538])

        # close gripper 
        self.move_gripper(-0.010)

        # arm home pose
        self.move_arm([0.0, -1.0, 0.3, 0.7])

	def search(self, s):
        m = s.data.split()
        if m[0] == 'bottle' or m[0] == 'vase': # msg converter sends only this msgs but anyway check it 
            self.num_detected += 1					
            img_mid = 640
            dif = int(m[2]) - int(m[1])
            bottle_mid = int(m[1]) + (dif / 2)
            threshold = 30
            ang_vel = 0.15
            vel = 0.15

            rospy.loginfo(dif)
            #rospy.loginfo(box.Class)
            #rospy.loginfo(rospy.get_time())

            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            if self.num_detected == 1:
                self.pub.publish(twist)

            if img_mid > bottle_mid + threshold:				
                twist.angular.z = ang_vel
                        
            elif img_mid < bottle_mid - threshold:
                twist.angular.z = -ang_vel
            else:
                twist.angular.z = 0
            
            if dif < 100:
                twist.linear.x = vel

            elif dif < 200:
                twist.linear.x = vel - 0.05
            elif dif < 290:
                twist.linear.x = vel-0.1

            else:
                twist.linear.x = 0
                self.stop += 1		
            
            self.bottle_pub.publish(True)
            #rospy.loginfo(box)
            self.pub.publish(twist)
            if self.stop==2:
                self.stop += 1
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                self.pub.publish(twist)
                rospy.sleep(2)
                ##Pick it up
                self.pick()
			

def main(args):
	rospy.init_node('task3_yolo', anonymous=True)
	sm = simple_motion()
	#sm.rotate()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("shutting_down")

if __name__ == '__main__':
	main(sys.argv)
