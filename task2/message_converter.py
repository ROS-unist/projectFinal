#!/usr/bin/env python
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String
import rospy

class m_converter:
    def __init__(self):
        self.sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback_fn)
        self.pub = rospy.Publisher('converted_message', String, queue_size=20)

    def callback_fn(self, data):
        '''
        convert bounding box msg to string msg
        '''
        for box in data.bounding_boxes:
            '''
            float64 probability
            int64 xmin
            int64 ymin
            int64 xmax
            int64 ymax
            int16 id
            string Class
            '''
            if box.Class == 'bottle' or box.Class == 'vase':
                self.pub.publish(box.Class + ' ' + str(box.xmin) + ' ' + str(box.xmax))

def main():
    c = m_converter()
    rospy.init_node('m_converter', anonymous=True)
    rospy.spin()

if __name__=='__main__':
    main()