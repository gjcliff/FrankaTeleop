import os
from typing import Any

import cv2
import numpy as np
from ament_index_python import get_package_share_directory
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from handcv.drawing_tools import (
    HAND_CONNECTIONS,
    NormalizedLandmark,
    NormalizedLandmarkList,
    draw_landmarks,
    get_default_hand_connections_style,
    get_default_hand_landmarks_style,
)


class MediaPipeRos:
    def __init__(self):
        self.MARGIN = 10  # pixels
        self.FONT_SIZE = 1
        self.FONT_THICKNESS = 1
        self.HANDEDNESS_TEXT_COLOR = (88, 205, 54)  # vibrant green
        self.gesture_recognizer = self.initialize_mediapipe()

    def draw_landmarks_on_image(self, rgb_image, detection_result, logger):
        hand_landmarks_list = detection_result.hand_landmarks
        handedness_list = detection_result.handedness
        annotated_image = np.copy(rgb_image)

        # loop through the detected hands to visualize.
        for idx in range(len(hand_landmarks_list)):
            hand_landmarks = hand_landmarks_list[idx]
            handedness = handedness_list[idx]

            # draw the hand landmarks.
            hand_landmarks_new_list = NormalizedLandmarkList()
            hand_landmarks_new_list.landmark.extend(
                [
                    NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z)
                    for landmark in hand_landmarks
                ]
            )
            draw_landmarks(
                logger=logger,
                image=annotated_image,
                landmark_list=hand_landmarks_new_list,
                connections=HAND_CONNECTIONS,
                landmark_drawing_spec=get_default_hand_landmarks_style(),
                connection_drawing_spec=get_default_hand_connections_style(),
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
            # cv2.imshow("window", annotated_image)
            # cv2.waitKey(1)

        return annotated_image

    def initialize_mediapipe(self) -> Any:
        # initialize the mediapipe task file's path
        self.model_path = os.path.join(
            get_package_share_directory("handcv"),
            "config/gesture_recognizer.task",
        )

        base_options = python.BaseOptions(model_asset_path=self.model_path)
        options = vision.GestureRecognizerOptions(
            base_options=base_options, num_hands=2
        )
        recognizer = vision.GestureRecognizer.create_from_options(options)

        return recognizer
