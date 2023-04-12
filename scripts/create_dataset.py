#!/usr/bin/env python3
import time
import os
import csv
import random
from std_msgs.msg import Int8MultiArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from skimage.transform import resize
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import rospy
import roslib
import sys
import numpy as np

# Set capture hz <recommended same as imitation learning>
HZ = 5

# cmd_vel topic
CMD_VEL_TOPIC = "/nav_vel"

class create_dataset():
    def __init__(self):
        rospy.init_node('create_dataset_node', anonymous=True)
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.callback)
        self.image_left_sub = rospy.Subscriber(
            "/camera_left/rgb/image_raw", Image, self.callback_left_camera)
        self.image_right_sub = rospy.Subscriber(
            "/camera_right/rgb/image_raw", Image, self.callback_right_camera)
        self.cmd = rospy.Subscriber(
            CMD_VEL_TOPIC, Twist, self.callback_cmd)
        self.dir_cmd_sub = rospy.Subscriber(
            "/cmd_dir", Int8MultiArray, self.callback_cmd_data)
        self.target_action = 0.0
        self.counter = 0
        self.bridge = CvBridge()
        self.path = roslib.packages.get_pkg_dir(
            'create_dataset') + '/dataset/'
        os.makedirs(self.path + self.start_time + '/image/center')
        os.makedirs(self.path + self.start_time + '/image/left')
        os.makedirs(self.path + self.start_time + '/image/right')
        self.image_path = roslib.packages.get_pkg_dir(
            'create_dataset') + '/dataset/' + self.start_time + '/image/'
        self.img_center_path = self.image_path + '/center/'
        self.img_left_path = self.image_path + '/left/'
        self.img_right_path = self.image_path + '/right/'
        self.cmd_path = roslib.packages.get_pkg_dir(
            'create_dataset') + '/dataset/' + self.start_time + '/target_action'
        self.dir_cmd_data = (100, 0, 0)
        self.flag = False
        self.old_img = np.zeros((48, 64, 3))

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.flag = True
        except CvBridgeError as e:
            print(e)

    def callback_left_camera(self, data):
        try:
            self.cv_left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_right_camera(self, data):
        try:
            self.cv_right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_cmd(self, data):
        self.target_action = data.angular.z

    def callback_cmd_data(self, data):
        self.dir_cmd_data = data.data

    def loop(self):
        if not self.flag:
            return
        if self.cv_image.size != 640 * 480 * 3:
            return
        if self.cv_left_image.size != 640 * 480 * 3:
            return
        if self.cv_right_image.size != 640 * 480 * 3:
            return

        img = cv2.resize(self.cv_image, (64, 48))
        r, g, b = cv2.split(img)
        imgobj = np.asanyarray([r, g, b])

        img_left = cv2.resize(self.cv_left_image, (64, 48))
        r, g, b = cv2.split(img_left)
        imgobj_left = np.asanyarray([r, g, b])

        img_right = cv2.resize(self.cv_right_image, (64, 48))
        r, g, b = cv2.split(img_right)
        imgobj_right = np.asanyarray([r, g, b])

        ros_time = str(rospy.Time.now())

        if np.allclose(self.old_img, img):
            print("\nwritten path:", self.path + self.start_time)
            disk = 6.2 * 3 * self.counter * 0.001
            print("projected disk usage: " + f'{disk:.3f}' + "MB")
            print("completed successfully")
            sys.exit()

        line = ["create_dataset_mode", str(self.counter), str(self.target_action), str(self.dir_cmd_data)]
        with open(self.cmd_path + '.csv', 'a') as f:
            writer = csv.writer(f, lineterminator='\n')
            writer.writerow(line)

        cv2.imwrite(self.img_center_path + "center" + str(self.counter) + ".png", img)
        cv2.imwrite(self.img_left_path + "left" + str(self.counter) + ".png", img_left)
        cv2.imwrite(self.img_right_path + "right" + str(self.counter) + ".png", img_right)

        print("capture_count:", self.counter)

        self.old_img = img

        self.counter += 1

if __name__ == '__main__':
    rg = create_dataset()
    DURATION = float(1) / HZ
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()