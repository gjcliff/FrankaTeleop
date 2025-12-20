#!/usr/bin/env python3

import mediapipe as mp

from handcv.drawing_tools import (
    HAND_CONNECTIONS,
    get_default_hand_connections_style,
    get_default_hand_landmarks_style,
    draw_landmarks,
)
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import cv2
import os
import dataclasses

from ament_index_python import get_package_share_directory


@dataclasses.dataclass
class NormalizedLandmark:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclasses.dataclass
class NormalizedLandmarkList:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class MediaPipeRos:

    def __init__(self):
        self.MARGIN = 10  # pixels
        self.FONT_SIZE = 1
        self.FONT_THICKNESS = 1
        self.HANDEDNESS_TEXT_COLOR = (88, 205, 54)  # vibrant green
        self.landmarker = self.initialize_mediapipe()

    def draw_landmarks_on_image(self, rgb_image, detection_result):
        hand_landmarks_list = detection_result.hand_landmarks
        handedness_list = detection_result.handedness
        annotated_image = np.copy(rgb_image)

        # Loop through the detected hands to visualize.
        for idx in range(len(hand_landmarks_list)):
            hand_landmarks = hand_landmarks_list[idx]
            handedness = handedness_list[idx]

            # Draw the hand landmarks.
            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            hand_landmarks_proto.landmark.extend(
                [
                    landmark_pb2.NormalizedLandmark(
                        x=landmark.x, y=landmark.y, z=landmark.z
                    )
                    for landmark in hand_landmarks
                ]
            )
            draw_landmarks(
                annotated_image,
                hand_landmarks_proto,
                HAND_CONNECTIONS,
                get_default_hand_landmarks_style(),
                get_default_hand_connections_style(),
            )

            # Get the top left corner of the detected hand's bounding box.
            height, width, _ = annotated_image.shape
            x_coordinates = [landmark.x for landmark in hand_landmarks]
            y_coordinates = [landmark.y for landmark in hand_landmarks]
            text_x = int(min(x_coordinates) * width)
            text_y = int(min(y_coordinates) * height) - self.MARGIN

            # Draw handedness (left or right hand) on the image.
            cv2.putText(
                annotated_image,
                f"{handedness[0].category_name}",
                (text_x, text_y),
                cv2.FONT_HERSHEY_DUPLEX,
                self.FONT_SIZE,
                self.HANDEDNESS_TEXT_COLOR,
                self.FONT_THICKNESS,
                cv2.LINE_AA,
            )

        return annotated_image

    def initialize_mediapipe(self):
        # initialize the mediapipe task file's path
        self.model_path = os.path.join(
            get_package_share_directory("handcv"),
            "config/gesture_recognizer.task",
        )

        BaseOptions = mp.tasks.BaseOptions
        GestureRecognizer = mp.tasks.vision.GestureRecognizer
        GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        options = GestureRecognizerOptions(
            base_options=BaseOptions(self.model_path),
            running_mode=VisionRunningMode.IMAGE,
            num_hands=2,
        )

        recognizer = GestureRecognizer.create_from_options(options)

        return recognizer

    def do_nothing(self):
        pass
