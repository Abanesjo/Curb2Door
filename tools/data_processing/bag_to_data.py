#!/usr/bin/env python
import rosbag
import numpy as np
from tqdm import tqdm
import os
import cv2
from cv_bridge import CvBridge

bag_filename = '/mnt/p/curb2door/bag/r3live/r3live_loop2_undistorted.bag'
output_dir = '/mnt/p/curb2door/bag/extracted_data/r3live_loop2_undistorted'
odom_file = os.path.join(output_dir, 'odom.txt')

bridge = CvBridge()

if __name__ == '__main__':
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    if not os.path.exists(os.path.join(output_dir, 'img')):
        os.makedirs(os.path.join(output_dir, 'img'))

    with rosbag.Bag(bag_filename, 'r') as bag:
        with open(odom_file, 'w') as f:
            for topic, msg, t in tqdm(bag.read_messages()):
                if topic=='/track_img':
                    try:
                        image = bridge.imgmsg_to_cv2(msg,   desired_encoding='passthrough')
                        cv2.imwrite(os.path.join(output_dir, 'img', f'{t}.png'), image)
                    except Exception as e:
                        print(e)
                        continue
                if topic=='/camera_odom':
                    f.write(f'{t} {msg.pose.pose.position.x} {msg.pose.pose.position.y} {msg.pose.pose.position.z} {msg.pose.pose.orientation.x} {msg.pose.pose.orientation.y} {msg.pose.pose.orientation.z} {msg.pose.pose.orientation.w}\n')
