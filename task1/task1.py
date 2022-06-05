#!/usr/bin/env python
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
import rospy

IMAGE_SIZE_X = 1280
CRITICAL_BOX_SIZE = 1280 / 10

class task1:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.callback_fn)
    
    def callback_fn(self, data):
        for box in data.bounding_boxes:
            if box.Class == 'bottle':

                '''
                float64 probability
                int64 xmin
                int64 ymin
                int64 xmax
                int64 ymax
                int16 id
                string Class
                '''

                '''
                display size:
                1280 x 720 pixels
                '''

                # find middle of bottle's bounding box
                mid_x = (box.xmin + box.xmax)/2
                dis = IMAGE_SIZE_X/2 - mid_x

                twist = Twist()

                # move toward bottle until the size of its bounding box >= critical size
                if box.xmax - box.xmin < CRITICAL_BOX_SIZE:
                    twist.linear.x = 0.2; twist.linear.y = 0.0; twist.linear.z = 0.0
                else:
                    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0

                # if the center of bounding box is not at screen center with some error range, rotate
                if abs(dis) > 50:
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.2 if dis > 0 else -0.2
                else:
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.2 if dis > 0 else -0.2
                self.pub.publish(twist)

def main():
    c = task1()
    rospy.init_node('task1', anonymous=True)
    rospy.spin()


if __name__=="__main__":
    main()

