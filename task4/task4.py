#!/usr/bin/env python
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from std_msgs.msgs import Int32, Bool, String
import  sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class m4:
    def __init__(self) -> None:
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.callback_fn)
    
    def callback_fn(self, data):
        for box in data.bounding_boxes:
            if (box.Class == 'bottle'):
                midImage = 640
				difImage = box.xmax - box.xmin
				midBottle = box.xmin + (difImage / 2)
				e = 100
				vel = 0.15
				twist = Twist()
				twist.linear.x = 0
				twist.linear.y = 0
				twist.linear.z = 0
				twist.angular.x = 0
				twist.angular.y = 0
				if midImage > midBottle + e:
					twist.angular.z = vel
					
				elif midImage < midBottle - e:
					twist.angular.z = -vel
				else:
					twist.angular.z = 0
				self.pub.publish(twist)

def main():
    c = m4()
    rospy.init_node('m4', anonymous=True)
    rospy.spin()


if __name__=="__main__":
    main()
