#!/usr/bin/env python
from cv_bridge import CvBridge
import os
import cv2 as cv

from sensor_msgs.msg import Image
from tm_msgs.srv import *
from tm_msgs.msg import *
import rclpy

from rclpy.node import Node

import sys
sys.path.append(
    '/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
sys.path.append('/home/robot/workspace2/team2_ws/src/send_script/send_script')

from ImageLoader import Calibration
from Gripper import ArmControl, Gripper

IMG_PATH = '/home/robot/workspace2/team2_ws/src/send_script/send_script/calibration'


def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()


def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')

    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()


class ImageSub(Node):
    id = 0
    num_photo = 30
    calibration = Calibration()
    arm = ArmControl(send_script, set_io)

    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image,
                                                     'techman_image', self.image_callback, 10)
        self.subscription

    @staticmethod
    def check_calibrate(img):
        if ImageSub.id == ImageSub.num_photo:
            # Calibrate extrinsic

            ImageSub.calibration.init_extrinsic(
                True, arm=ImageSub.arm, extrinsic_img=img)
            print("===========Extrinsic Calibration result===========")
            print('rvec: ', ImageSub.calibration.rvec)
            print('tvec: ', ImageSub.calibration.tvec)
            return True
        elif ImageSub.id == ImageSub.num_photo-1:
            # Calibrate intrinsic
            ImageSub.calibration.init_intrinsic(img_path=IMG_PATH)
            print("===========Intrinsic Calibration result===========")
            print('K:    ', ImageSub.calibration.K)
            print('dist: ', ImageSub.calibration.dist)
            ImageSub.calibration.setup_bricks(ImageSub.arm)
            send_script("Vision_DoJob(job1)")
            return True
        else:
            return False

    def image_callback(self, data):
        self.get_logger().info('Received image')

        # TODO (write your code here)
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self.img = img

        if not os.path.isdir(IMG_PATH):
            os.mkdir(IMG_PATH)

        cv.imwrite(os.path.join(IMG_PATH, str(id) + '.png'), img)
        ImageSub.id += 1
        if not ImageSub.check_calibrate(img):
            send_script("Vision_DoJob(job1)")
        print("===========Calibration finished===========")


def main(args=None):
    rclpy.init(args=args)

    node = ImageSub('image_sub')
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
