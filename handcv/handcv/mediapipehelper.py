import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import cv2
import os

from ament_index_python import get_package_share_directory


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
            hand_landmarks_proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
            ])
            solutions.drawing_utils.draw_landmarks(
                annotated_image,
                hand_landmarks_proto,
                solutions.hands.HAND_CONNECTIONS,
                solutions.drawing_styles.get_default_hand_landmarks_style(),
                solutions.drawing_styles.get_default_hand_connections_style())

            # Get the top left corner of the detected hand's bounding box.
            height, width, _ = annotated_image.shape
            x_coordinates = [landmark.x for landmark in hand_landmarks]
            y_coordinates = [landmark.y for landmark in hand_landmarks]
            text_x = int(min(x_coordinates) * width)
            text_y = int(min(y_coordinates) * height) - self.MARGIN

            # Draw handedness (left or right hand) on the image.
            cv2.putText(annotated_image, f"{handedness[0].category_name}",
                        (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX,
                        self.FONT_SIZE, self.HANDEDNESS_TEXT_COLOR, self.FONT_THICKNESS, cv2.LINE_AA)

        return annotated_image

    def initialize_mediapipe(self):
        # initialize the mediapipe task file's path
        self.model_path = os.path.join(
            get_package_share_directory('handcv'), 'config/gesture_recognizer.task')

        # mediapipe variables
        # BaseOptions = mp.tasks.BaseOptions
        # HandLandmarker = mp.tasks.vision.HandLandmarker
        # HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
        # VisionRunningMode = mp.tasks.vision.RunningMode
        #
        # options = HandLandmarkerOptions(
        #     base_options=BaseOptions(self.model_path),
        #     num_hands=2,
        #     running_mode=VisionRunningMode.IMAGE,
        #     # result_callback=self.do_nothing # only include if in LIVE_STEAM vision running mode
        # )

        # base_options = python.BaseOptions(model_asset_path=self.model_path)
        # options = vision.GestureRecognizerOptions(base_options=base_options,
        #                                           running_mode=vision.RunningMode.IMAGE,
        #                                           num_hands=2)
        # recognizer = vision.GestureRecognizer.create_from_options(options)

        BaseOptions = mp.tasks.BaseOptions
        GestureRecognizer = mp.tasks.vision.GestureRecognizer
        GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        # Create a gesture recognizer instance with the image mode:
        options = GestureRecognizerOptions(base_options=BaseOptions(self.model_path),
                                           running_mode=VisionRunningMode.IMAGE,
                                           num_hands=2)

        recognizer = GestureRecognizer.create_from_options(options)

        return recognizer

    def do_nothing(self):
        pass
