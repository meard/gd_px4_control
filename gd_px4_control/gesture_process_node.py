#!/usr/bin/env python3
import sys
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
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.components.containers import NormalizedLandmark


class GestureDetect(Node):

    def __init__(self):
        super().__init__('gesture_process_node')

        VisionRunningMode = vision.RunningMode

        base_options = python.BaseOptions(
            model_asset_path='model/hand_landmarker.task')
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=2,
            running_mode=VisionRunningMode.VIDEO,
        )

        self.detector = vision.HandLandmarker.create_from_options(options)

        self.mpDraw = mp.solutions.drawing_utils
        self._img_msg = Image()
        self.br = CvBridge()

        self._image_sub = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)

        timer_period = 0.1     # seconds
        self.counter = 0       # counter
        gdetect = self.create_timer(timer_period, self.main)

    def image_callback(self, msg):
        self._img_msg = self.br.imgmsg_to_cv2(msg)

    @staticmethod
    def convert_landmarks_to_image_coordinates(
        # type: ignore
        hand_landmarks: list[list[NormalizedLandmark]], width: int, height: int
    ) -> list[tuple[int, int]]:
        return [(int(lm.x * width), int(lm.y * height)) for hand_landmark in hand_landmarks for lm in hand_landmark]

    def findHands(self, img: np.ndarray[np.uint8]) -> None:
        img = self._img_msg
        height, width, _ = img.shape
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img)
        recognition_result = self.detector.detect(image)
        if recognition_result.hand_landmarks:
            hand_landmarks = recognition_result.hand_landmarks
            landmark_positions_3d = self.convert_landmarks_to_3d(
                hand_landmarks)
            if landmark_positions_3d is not None:
                self._logger.error("No Ladnmark found")

            # Convert normalized coordinates to image coordinates
            points = self.convert_landmarks_to_image_coordinates(
                hand_landmarks, width, height)

            # Obtain hand connections from MediaPipe
            mp_hands_connections = mp.solutions.hands.HAND_CONNECTIONS
            points1 = [points[connection[0]]
                       for connection in mp_hands_connections]
            points2 = [points[connection[1]]
                       for connection in mp_hands_connections]

            return recognition_result

        else:
            return False

    def main(self):
        detect_results = self.findHands(self._img_msg)

        if detect_results:
            print('hand detected')
            annotated_image = self.mpDraw.draw_landmarks(
                self._img_msg, detect_results)
            cv2.imshow("Image", annotated_image)
        else:
            print('no hand detected')
            cv2.imshow("Image", self._img_msg)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Closing Camera Stream")
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
