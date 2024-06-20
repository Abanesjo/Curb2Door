#!/usr/bin/env python
import os
import rosbag
import cv2
from cv_bridge import CvBridge
import argparse
import numpy as np

def extract_images(bag_file, output_dir, image_topic):
    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Initialize the CvBridge class
    bridge = CvBridge()

    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[image_topic]):
            # Convert the ROS Image message to a OpenCV image
            # cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Save the image with the timestamp as the filename
            timestamp = str(t.to_nsec())
            image_filename = os.path.join(output_dir, f"{timestamp}.png")
            cv2.imwrite(image_filename, cv_image)

            print(f"Saved {image_filename}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag file.")
    parser.add_argument("bag_file", help="Input ROS bag file")
    parser.add_argument("output_dir", help="Output directory to save images")
    parser.add_argument("image_topic", help="Image topic to extract images from")

    args = parser.parse_args()

    extract_images(args.bag_file, args.output_dir, args.image_topic)
