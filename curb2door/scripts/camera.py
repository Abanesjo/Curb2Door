#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def camera_publisher():
    rospy.init_node('camera_node', anonymous=True)
    bridge = CvBridge()

    # Publishers for manually compressed images
    front_image_pub = rospy.Publisher('front_camera_image/compressed', CompressedImage, queue_size=10)
    back_image_pub = rospy.Publisher('back_camera_image/compressed', CompressedImage, queue_size=10)

    cap = cv2.VideoCapture(0, )  # Adjust as needed for your camera setup
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3008)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1504)

    rate = rospy.Rate(10)  # Adjust the rate as per your requirements

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            # Capture the timestamp as soon as the frame is captured
            current_time = rospy.Time.now()

            # Split and process the frame as needed
            h, w, c = frame.shape
            half = w // 2
            left_part = frame[:, :half]
            right_part = frame[:, half:]

            # Compress the images using OpenCV
            encode_start = rospy.Time.now()
            ret, left_jpeg = cv2.imencode('.jpg', left_part)
            encode_end = rospy.Time.now()
            ret, right_jpeg = cv2.imencode('.jpg', right_part)

            # Prepare the CompressedImage messages
            pub_start = rospy.Time.now()
            front_img_msg = CompressedImage()
            front_img_msg.header.stamp = current_time  # Use the captured timestamp
            front_img_msg.header.frame_id = "front_camera_frame"
            front_img_msg.format = "jpeg"
            front_img_msg.data = left_jpeg.tobytes()

            back_img_msg = CompressedImage()
            back_img_msg.header.stamp = current_time  # Use the captured timestamp
            back_img_msg.header.frame_id = "back_camera_frame"
            back_img_msg.format = "jpeg"
            back_img_msg.data = right_jpeg.tobytes()

            # Publish the compressed images
            front_image_pub.publish(front_img_msg)
            pub_end = rospy.Time.now()
            back_image_pub.publish(back_img_msg)
            everything_end = rospy.Time.now()

        rate.sleep()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
