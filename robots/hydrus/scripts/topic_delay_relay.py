#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

class TopicDelayRelay:
    def __init__(self):
        rospy.init_node('topic_delay_relay', anonymous=True)
        self.sub = rospy.Subscriber('/hydrus/ground_truth', Odometry, self.gtCallback)
        self.pub = rospy.Publisher('/hydrus/ground_truth_delayed', Odometry, queue_size = 1)

        self.delay_count = rospy.get_param('~delay_count', 20)

        self.msg_queue = []

    def gtCallback(self, msg):
        self.msg_queue.append(msg)

        if len(self.msg_queue) <= self.delay_count:
            return

        out_msg = self.msg_queue.pop(0)
        out_msg.header.stamp = rospy.Time.now()

        self.pub.publish(out_msg)


if __name__ == '__main__':
    node = TopicDelayRelay()
    rospy.spin()
