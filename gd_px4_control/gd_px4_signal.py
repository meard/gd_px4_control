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

from gd_px4_control.utils.hand import HandDetector
from gd_px4_control.utils.templates import Gesture
from gd_px4_control.utils.utils import two_landmark_distance
from gd_px4_control.utils.utils import calculate_angle, calculate_thumb_angle, get_finger_state
from gd_px4_control.utils.utils import map_gesture, draw_bounding_box, draw_fingertips


THUMB_THRESH = [9, 8]
NON_THUMB_THRESH = [8.6, 7.6, 6.6, 6.1]

BENT_RATIO_THRESH = [0.76, 0.88, 0.85, 0.65]

CAM_W = 640
CAM_H = 480
TEXT_COLOR = (243, 236, 27)


class gesturePublisher(Node):

    def __init__(self):
        super().__init__('gesture_publisher')

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
        self.mode = 'single'
        self.target_gesture = 'all'

        self.br = CvBridge()
        self.hand_detector = HandDetector(False, 2, 1, 0.5)

        # ROS 2 Publishers, Subscribers
        self.pub_gesture = self.create_publisher(
            String, 'gesture_signal', 10)

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # ****************************************************************************************
    def check_finger_states(self, hand):
        landmarks = hand['landmarks']
        label = hand['label']
        facing = hand['facing']

        self.finger_states = [None] * 5
        joint_angles = np.zeros((5, 3))  # 5 fingers and 3 angles each

        # wrist to index finger mcp
        d1 = two_landmark_distance(landmarks[0], landmarks[5])

        for i in range(5):
            joints = [0, 4*i+1, 4*i+2, 4*i+3, 4*i+4]
            if i == 0:
                joint_angles[i] = np.array(
                    [calculate_thumb_angle(
                        landmarks[joints[j:j+3]], label, facing) for j in range(3)]
                )
                self.finger_states[i] = get_finger_state(
                    joint_angles[i], THUMB_THRESH)
            else:
                joint_angles[i] = np.array(
                    [calculate_angle(landmarks[joints[j:j+3]])
                     for j in range(3)]
                )
                d2 = two_landmark_distance(
                    landmarks[joints[1]], landmarks[joints[4]])
                self.finger_states[i] = get_finger_state(
                    joint_angles[i], NON_THUMB_THRESH)

                if self.finger_states[i] == 0 and d2/d1 < BENT_RATIO_THRESH[i-1]:
                    self.finger_states[i] = 1

        return self.finger_states

    def detect_gesture(self, img, draw=True):
        hands = self.hand_detector.detect_hands(img)
        self.detected_gesture = None

        if hands:
            if self.mode == 'single':
                hand = hands[-1]
                self.check_finger_states(hand)
                if draw:
                    self.draw_gesture_landmarks(img)

                ges = Gesture(hand['label'])
                self.detected_gesture = map_gesture(ges.gestures,
                                                    self.finger_states,
                                                    hand['landmarks'],
                                                    hand['wrist_angle'],
                                                    hand['direction'],
                                                    hand['boundary'])

            if self.mode == 'double' and len(hands) == 2:
                pass

        return self.detected_gesture

    def draw_gesture_landmarks(self, img):
        hand = self.hand_detector.decoded_hands[-1]
        self.hand_detector.draw_landmarks(img)
        draw_fingertips(hand['landmarks'], self.finger_states, img)

    def draw_gesture_box(self, img):
        hand = self.hand_detector.decoded_hands[-1]
        draw_bounding_box(hand['landmarks'], self.detected_gesture, img)
    # ****************************************************************************************

    # ****************************************************************************************
    def timer_callback(self):
        """
            Main Function
        """
        _, img = self.cap.read()
        img = cv2.flip(img, 1)
        detected_gestures = self.detect_gesture(img, 2)
        if detected_gestures:
            if self.target_gesture == 'all' or self.target_gesture == detected_gestures:
                self.draw_gesture_box(img)
                self.pub_gesture_data.data = self.detected_gesture
                self.pub_gesture.publish(self.pub_gesture_data)
                print(self.detected_gesture)

        self.ctime = time.time()
        fps = 1 / (self.ctime - self.ptime)
        self.ptime = self.ctime

        cv2.putText(img, f'FPS: {int(fps)}', (50, 50), 0, 0.8,
                    TEXT_COLOR, 2, lineType=cv2.LINE_AA)

        cv2.imshow('Gesture detection', img)
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
