#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from PySide6.QtCore import Signal, QObject
from PySide6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
import sys
import time
import threading
import numpy as np
import cv2

class TopicMonitor(QObject):
    rate_signal = Signal(float)
    dimension_signal = Signal(int, int)
    
    def __init__(self, topic_name, message_type):
        super().__init__()
        rospy.init_node("topic_monitor")
        self.topic_name = topic_name
        self.message_type = message_type
        self.last_time = None
        self.msg_count = 0
        self.rate = 0
        self.start_time = time.time()
        self.last_print_time = self.start_time  # Initialize last print time
        self.subscriber = rospy.Subscriber(topic_name, message_type, self.callback)
        self.image_shape = (0, 0)

    def callback(self, msg):
        current_time = time.time()
        if self.last_time is not None:
            interval = current_time - self.last_time
            rate = 1.0 / interval
        self.last_time = current_time
        self.msg_count += 1
        elapsed_time = current_time - self.start_time

        # Emit the average rate every second
        if current_time - self.last_print_time >= 1.0:
            if elapsed_time > 0:
                avg_rate = self.msg_count / elapsed_time
                # print(f"Average Rate (): {avg_rate:.2f} Hz")
                self.rate_signal.emit(avg_rate)  # Emit the signal with the average rate
            self.last_print_time = current_time  # Update last print time
        
        if self.message_type == CompressedImage:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            height, width, _ = image.shape
            self.dimension_signal.emit(width, height)


def start_ros_spin():
    rospy.spin()

if __name__ == "__main__":
    # Create the Qt application
    app = QApplication(sys.argv)

    # Create a QWidget to display the rate
    window = QWidget()
    layout = QVBoxLayout()
    rate_label = QLabel("Rate: 0.00 Hz")
    layout.addWidget(rate_label)
    window.setLayout(layout)
    window.show()

    # Create the TopicMonitor and connect the signal
    topic_monitor = TopicMonitor("/front_camera_image/compressed", CompressedImage)
    topic_monitor.rate_signal.connect(lambda rate: rate_label.setText(f"Rate: {rate:.2f} Hz"))

    # Start the ROS spin in a separate thread
    threading.Thread(target=start_ros_spin).start()

    # Run the Qt application
    sys.exit(app.exec())
