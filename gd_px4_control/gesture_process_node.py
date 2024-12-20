#!/usr/bin/env python3
import sys
import os
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
# from cvzone.HandTrackingModule import HandDetector
import numpy as np

import mediapipe as mp
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.vision import GestureRecognizer, GestureRecognizerResult, GestureRecognizerOptions
from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerResult, HandLandmarkerOptions
from mediapipe.tasks.python.components.containers import NormalizedLandmark


class GestureDetect(Node):

    def __init__(self):
        super().__init__('gesture_process_node')

        self.pTime = 0
        self.cTime = 0

        self.br = CvBridge()
        self._img_msg = None
        self.mp_hand = solutions.hands
        self.mp_hand_detect = solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        self.mp_drawing = solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.results = []

        self._image_sub = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)

        timer_period = 0.1     # seconds
        self.counter = 0       # counter
        gdetect = self.create_timer(timer_period, self.gd_main)

    def image_callback(self, msg):
        self._img_msg = self.br.imgmsg_to_cv2(msg)

    def prepare_frame(self, in_image):
        # Show the FPS
        self.cTime = time.time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime

        cv2.putText(in_image, str(int(fps)), (10, 70),
                    cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
        return in_image

    def detect_hands(self, in_image, hand_results):

        if hand_results.multi_hand_landmarks:
            for handLms in hand_results.multi_hand_landmarks:

                for id, lm in enumerate(handLms.landmark):
                    print(id, lm)
                    # h, w, c = in_image.shape
                    # cx, cy = int(lm.x*w), int(lm.y*h)

                    # if id == 4 or id == 8:
                    #     cv2.circle(in_image, (cx, cy), 25,
                    #                (255, 0, 255), cv2.FILLED)

                self.mp_drawing.draw_landmarks(
                    in_image, handLms, self.mp_hand.HAND_CONNECTIONS)

        in_image = self.prepare_frame(in_image)
        return in_image

    def gd_main(self):

        if self._img_msg is not None:

            img_frame = cv2.cvtColor(cv2.resize(
                self._img_msg, (640, 480)), cv2.COLOR_BGR2RGB)

            hand_results = self.mp_hand_detect.process(img_frame)
            annotated_frame = self.detect_hands(img_frame, hand_results)

            cv2.imshow('Show', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Closing Camera Stream")
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)

        self.counter += 1


def main(args=None):
    """
    Run a Listener node standalone.

    This function is called directly when using an entrypoint. Entrypoints are configured in
    setup.py. This along with the script installation in setup.cfg allows a listener node to be run
    with the command `ros2 run examples_rclpy_executors listener`.

    :param args: Arguments passed in from the command line.
    """
    try:
        rclpy.init(args=args)
        bD = GestureDetect()
        rclpy.spin(bD)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    # Runs a listener node when this script is run directly (not through an entrypoint)
    main()
