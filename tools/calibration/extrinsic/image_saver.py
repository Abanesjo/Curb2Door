#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import os

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)
        self.image_sub = rospy.Subscriber("/back_camera_image/compressed", CompressedImage, self.callback)
        self.frame_count = 0
        self.save_path = os.getcwd()  # Get the current directory

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)  # Updated to np.frombuffer for better compatibility
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        filename = os.path.join(self.save_path, "data", "images", f"frame_{self.frame_count:04d}.jpg")
        cv2.imwrite(filename, image_np)
        rospy.loginfo(f"Saved {filename}")
        self.frame_count += 1

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    saver = ImageSaver()
    saver.run()
