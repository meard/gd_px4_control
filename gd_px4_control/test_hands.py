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

# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()


class GestureDetect(Node):

    def __init__(self):
        super().__init__('gesture_process_node')

        self.br = CvBridge()
        self._img_msg = None
        self.mp_drawing = solutions.drawing_utils
        self.mp_hand = solutions.hands
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.results = []

        base_options = python.BaseOptions(
            model_asset_path='hand_landmarker.task'
        )

        options = HandLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.LIVE_STREAM,
            num_hands=2,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
            result_callback=self.print_result

        )

        self.recognizer = vision.HandLandmarker.create_from_options(options)

        self._image_sub = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)

        timer_period = 0.1     # seconds
        self.counter = 0       # counter
        gdetect = self.create_timer(timer_period, self.gd_main)

    def image_callback(self, msg):
        self._img_msg = self.br.imgmsg_to_cv2(msg)

    def print_result(self, result: HandLandmarkerResult,
                     output_image: mp.Image, timestamp_ms: int):
        global FPS, COUNTER, START_TIME

        # Calculate the FPS
        if COUNTER % 10 == 0:
            FPS = 10 / (time.time() - START_TIME)
            START_TIME = time.time()

        self.results.append(result)
        COUNTER += 1

    # def prepare_frame(self):
    #      # Show the FPS
    #     fps_text = 'FPS = {:.1f}'.format(FPS)
    #     text_location = (24, 50)
    #     current_frame = in_image
    #     cv2.putText(current_frame, fps_text, text_location, cv2.FONT_HERSHEY_DUPLEX,
    #                 1, (0, 0, 0), 1, cv2.LINE_AA)
    #     pass

    def draw_landmark_on_frame(self, in_image):

        # in_image = cv2.flip(in_image, 1)
        fps_text = 'FPS = {:.1f}'.format(FPS)
        text_location = (24, 50)
        annotated_image = None
        current_frame = in_image
        cv2.putText(current_frame, fps_text, text_location, cv2.FONT_HERSHEY_DUPLEX,
                    1, (0, 0, 0), 1, cv2.LINE_AA)

        if self.results:
            for hand_index, hand_landmarks in enumerate(
                    self.results[0].hand_landmarks):
                # Calculate the bounding box of the hand
                # x_min = min([landmark.x for landmark in hand_landmarks])
                # y_min = min([landmark.y for landmark in hand_landmarks])
                # y_max = max([landmark.y for landmark in hand_landmarks])

                # # Convert normalized coordinates to pixel values
                # frame_height, frame_width = current_frame.shape[:2]
                # x_min_px = int(x_min * frame_width)
                # y_min_px = int(y_min * frame_height)
                # y_max_px = int(y_max * frame_height)

                # # Get gesture classification results
                # if self.results[0].hand_landmarks:
                #     gesture = self.results[0].gestures[hand_index]
                #     category_name = gesture[0].category_name
                #     score = round(gesture[0].score, 2)
                #     result_text = f'{category_name} ({score})'

                #     # Compute text size
                #     text_size = \
                #         cv2.getTextSize(result_text, cv2.FONT_HERSHEY_DUPLEX, 1,
                #                         2)[0]
                #     text_width, text_height = text_size

                #     # Calculate text position (above the hand)
                #     text_x = x_min_px
                #     text_y = y_min_px - 10  # Adjust this value as needed

                #     # Make sure the text is within the frame boundaries
                #     if text_y < 0:
                #         text_y = y_max_px + text_height

                #     # Draw the text
                #     cv2.putText(current_frame, result_text, (text_x, text_y),
                #                 cv2.FONT_HERSHEY_DUPLEX, 1,
                #                 (255, 255, 255), 2, cv2.LINE_AA)

                # Draw hand landmarks on the frame
                hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                hand_landmarks_proto.landmark.extend([
                    landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y,
                                                    z=landmark.z) for landmark in
                    hand_landmarks
                ])
                self.mp_drawing.draw_landmarks(
                    current_frame,
                    hand_landmarks_proto,
                    self.mp_hand.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style())

            annotated_image = current_frame
            self.results.clear()

            # landmark_list = detect_results.hand_landmarks
            # annotated_image = np.copy(in_image)
            # # Loop through the detected poses to visualize.
            # for idx in range(len(landmark_list)):
            #     hand_landmarks = landmark_list[idx]

            #     # Draw the pose landmarks.
            #     hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            #     hand_landmarks_proto.landmark.extend([
            #         landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
            #     ])
            #     solutions.drawing_utils.draw_landmarks(
            #         annotated_image,
            #         hand_landmarks_proto,
            #         solutions.hands.HAND_CONNECTIONS,
            #         solutions.drawing_styles.get_default_hand_connections_style())
        return annotated_image

    def gd_main(self):

        if self._img_msg is not None:

            img_frame = cv2.cvtColor(cv2.resize(
                self._img_msg, (640, 480)), cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB, data=img_frame)

            self.recognizer.detect_async(mp_image, self.counter)

            if (not (self.results is None)):
                annotated_frame = self.draw_landmark_on_frame(img_frame)

                # detect_frame = self.prepare_frame(self._img_msg)
                if annotated_frame is not None:
                    cv2.imshow('Hand Landmark Frame', annotated_frame)
                print("showing detected image")
            # else:
            #     cv2.imshow('Show', img_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Closing Camera Stream")
            self.recognizer.close()
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
