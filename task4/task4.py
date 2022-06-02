#!/usr/bin/env python
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
import rospy
import math
from time import sleep

class m4:
    def __init__(self) -> None:
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.callback_fn)
    
    def callback_fn(self, data):
        for box in data.bounding_boxes:
            twist = Twist()
            while (box.Class == 'bottle'):
                x1 = box.xmax
                y1 = box.ymax
                # twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
                e_dist = math.sqrt((x1 * x1) + (y1 * y1))
                
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.2
                self.pub.publish(twist)

            # wait and stop turtlebot
            sleep(5)
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            self.pub.publish(twist)
            


def main():
    c = m4()
    rospy.init_node('m4', anonymous=True)
    rospy.spin()


if __name__=="__main__":
    main()
