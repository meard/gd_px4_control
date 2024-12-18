#!/usr/bin/env python3
import sys
import os
import math

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
from mediapipe.tasks.python.components.containers import NormalizedLandmark


class GestureDetect(Node):

    def __init__(self):
        super().__init__('gesture_process_node')

        self.br = CvBridge()
        self._img_msg = None
        self.mp_drawing = solutions.drawing_utils
        self.mp_hand = solutions.hands
        self.results = None

        base_options = python.BaseOptions(
            model_asset_path='gesture_recognizer.task'
        )

        options = GestureRecognizerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.LIVE_STREAM,
            result_callback=self.print_result

        )

        self.recognizer = vision.GestureRecognizer.create_from_options(options)

        self._image_sub = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)

        timer_period = 0.1     # seconds
        self.counter = 0       # counter
        gdetect = self.create_timer(timer_period, self.gd_main)

    def image_callback(self, msg):
        self._img_msg = self.br.imgmsg_to_cv2(msg)

    def print_result(self, result: GestureRecognizerResult, output_image: mp.Image, timestamp_ms: int):
        # print('pose landmarker result: {}'.format(result))
        self.results = result
        # print(type(result))

    def draw_landmark_on_frame(self, in_image, detect_results):
        landmark_list = detect_results.hand_landmarks
        annotated_image = np.copy(in_image)
        # Loop through the detected poses to visualize.
        for idx in range(len(landmark_list)):
            hand_landmarks = landmark_list[idx]

            # Draw the pose landmarks.
            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            hand_landmarks_proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
            ])
            solutions.drawing_utils.draw_landmarks(
                annotated_image,
                hand_landmarks_proto,
                solutions.hands.HAND_CONNECTIONS,
                solutions.drawing_styles.get_default_hand_connections_style())
        return annotated_image

    def gd_main(self):

        if self._img_msg is not None:
            img_frame = self._img_msg
            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB, data=img_frame)

            self.recognizer.recognize_async(mp_image, self.counter)

            if(not (self.results is None)):
                annotated_frame = self.draw_landmark_on_frame(
                    mp_image.numpy_view(), self.results)

                cv2.imshow('Hand Landmark Frame',annotated_frame)
                print("showing detected image")
            else:
                cv2.imshow('Show', img_frame)

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
