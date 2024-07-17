#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time

class TopicMonitor:
    def __init__(self, topic_name, message_type):
        self.topic_name = topic_name
        self.message_type = message_type
        self.last_time = None
        self.msg_count = 0
        self.start_time = time.time()
        self.subscriber = rospy.Subscriber(topic_name, message_type, self.callback)

    def callback(self, msg):
        current_time = time.time()
        return_string = ""
        if self.last_time is not None:
            interval = current_time - self.last_time
            rate = 1.0 / interval
            return_string += f"Topic: {self.topic_name}, Rate: {rate:2.f} Hz |"
        self.last_time = current_time
        self.msg_count+=1
        elapsed_time = current_time - self.start_time
        if elapsed_time > 0:
            avg_rate = self.msg_count / elapsed_time
            return_string += f"Average Rate: {avg_rate:2f} Hz"

        print(return_string)
        return return_string
    