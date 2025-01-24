#!/usr/bin/python3

"""
Python node for publishing hand gesture output to PX4 control .

Author: Meard
Date: 20/01/2025

Requirements
* Make sure the external camera is connected.
"""

# Imports
import rclpy
import ament_index_python.packages
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import cvzone

import argparse
import numpy as np

from cvzone.HandTrackingModule import HandDetector

from gd_px4_control.utils.templates import Gesture

THUMB_THRESH = [9, 8]
NON_THUMB_THRESH = [8.6, 7.6, 6.6, 6.1]

BENT_RATIO_THRESH = [0.76, 0.88, 0.85, 0.65]

CAM_W = 640
CAM_H = 480
TEXT_COLOR = (243, 236, 27)


class gesturePublisher(Node):

    def __init__(self):
        super().__init__('twoHand_gesture_publisher')

        # global initilized variables
        self.img_data = Image()
        self.timestamp = Float64()
        self.pub_gesture_data = String()

        self.ptime = 0
        self.ctime = 0

        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, CAM_W)
        self.cap.set(4, CAM_H)

        self.detected_gesture = None

        self.br = CvBridge()
        self.hand_detector = HandDetector(detectionCon=0.8, maxHands=2)

        # ROS 2 Publishers, Subscribers
        self.pub_gesture = self.create_publisher(
            String, 'two_hand_gesture_signal', 10)

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # ****************************************************************************************
    def detect_gesture(self, img, hand):

        hand1 = hand[0]
        # List of 21 landmarks for the first hand
        lmList1 = hand1["lmList"]
        # Bounding box around the first hand (x,y,w,h coordinates)
        bbox1 = hand1["bbox"]
        center1 = hand1['center']  # Center coordinates of the first hand
        # Type of the first hand ("Left" or "Right")
        handType1 = hand1["type"]

        # Count the number of fingers up for the first hand
        fingers1 = self.hand_detector.fingersUp(hand1)

        if len(hand) == 2:
            hand2 = hand[1]
            lmList2 = hand2["lmList"]
            bbox2 = hand2["bbox"]
            center2 = hand2['center']
            handType2 = hand2["type"]

            # Count the number of fingers up for the second hand
            fingers2 = self.hand_detector.fingersUp(hand2)

            if fingers1 == [0, 0, 0, 0, 0] and fingers2 == [0, 0, 0, 0, 0]:
                # send HOLD signal
                pass
            if fingers1 == [0, 1, 0, 0, 0] and fingers2 == [1, 1, 1, 1, 1]:
                # send TAKEOFF signal
                pass
            if fingers1 == [0, 1, 1, 0, 0] and fingers2 == [0, 0, 0, 0, 0]:
                # send LAND signal
                pass
            if fingers1 == [0, 1, 1, 1, 0] and fingers2 == [0, 0, 0, 0, 0]:
                # send FORWARD signal
                pass
            if fingers1 == [0, 1, 1, 1, 1] and fingers2 == [0, 0, 0, 0, 0]:
                # send BACKWARD signal
                pass

    # ****************************************************************************************

    # ****************************************************************************************

    def detect_hands(self, img, draw=True):
        # hands = self.hand_detector.findHands(img, draw=False)
        hand, img = self.hand_detector.findHands(img, draw=True)

        if hand:
            self.detect_gesture(img, hand)

        return img

    # ****************************************************************************************

    # ****************************************************************************************

    # ****************************************************************************************

    # ****************************************************************************************

    def timer_callback(self):
        """
            Main Function
        """
        _, img = self.cap.read()

        detected_img = self.detect_hands(img)

        self.ctime = time.time()
        fps = 1 / (self.ctime - self.ptime)
        self.ptime = self.ctime

        cv2.putText(detected_img, f'FPS: {int(fps)}', (50, 50), 0, 0.8,
                    TEXT_COLOR, 2, lineType=cv2.LINE_AA)

        cv2.imshow('Gesture detection', detected_img)
        key = cv2.waitKey(1)
        if key == ord('q'):
            cv2.destroyAllWindows()
            self.cap.release()
            exit()

    # ****************************************************************************************


# # # # ========================================
# # # # ==========    Main
# # # # ========================================


def main(args=None):
    rclpy.init(args=args)

    gd = gesturePublisher()
    rclpy.spin(gd)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
