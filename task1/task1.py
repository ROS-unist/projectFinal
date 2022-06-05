#!/usr/bin/env python
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rospy

class task1:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.callback_fn)
        self.arm_pub = rospy.Publisher('move_arm', String, queue_size=1)
        self.rotate = True
        self.stop = 0
    def callback_fn(self, data):
        for box in data.bounding_boxes:
            if (box.Class == 'bottle') and self.stop <= 2:
                self.num_detected += 1
                midImage = 640 #to be configured
                difBoxes = box.xmax - box.xmin
                bottle_mid = box.xmin + (difBoxes / 2)
                threshold = 30
                velAngular = 0.15
                vel = 0.15
                #We will use this for calibration purpose
                # rospy.loginfo(difBoxes)
                # rospy.loginfo(box.Class)

                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                if self.num_detected == 1:
                    self.pub.publish(twist)
                if midImage > bottle_mid + threshold:
                    twist.angular.z = velAngular
                elif midImage < bottle_mid - threshold:
                    twist.angular.z = -velAngular
                else:
                    twist.angular.z = 0
				
                if difBoxes < 100:
                   twist.linear.x = vel

                elif difBoxes < 200:
                    twist.linear.x = vel - 0.05
                elif difBoxes < 290:
                    twist.linear.x = vel - 0.1

                else:
                    twist.linear.x = 0
                    self.stop += 1		
				
                self.pub.publish(True)
                #for calibration
				#rospy.loginfo(box)
                self.pub.publish(twist)
                if self.stop==2:
                    twist.angular.x = 0
                    twist.angular.y = 0
                    twist.angular.z = 0
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.linear.z = 0
                    self.pub.publish(twist)
                    rospy.sleep(0.5)
                    ##Pick it up
                    self.arm_pub.publish('G')
                    self.stop+=1

def main():
    c = task1()
    rospy.init_node('task1', anonymous=True)
    rospy.spin()


if __name__=="__main__":
    main()

