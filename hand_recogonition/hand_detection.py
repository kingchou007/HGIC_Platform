# Adapted code from:
# [hand-gesture-recognition-mediapipe]
# URL: [https://github.com/Kazuhito00/hand-gesture-recognition-using-mediapipe]
# License: [Apache v2 license.]

import cv2 as cv
import mediapipe as mp
import numpy as np
from timingdecorator.timeit import timeit


class HandDetector:
    def __init__(
        self,
        use_static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    ):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=use_static_image_mode,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

        self.mp_pose = mp.solutions.pose

        # drawing utilities
        self.mp_drawing = mp.solutions.drawing_utils
        self.landmark_drawing_spec = self.mp_drawing.DrawingSpec(
            color=(255, 0, 0), thickness=4, circle_radius=3
        )
        self.connection_drawing_spec = self.mp_drawing.DrawingSpec(
            color=(0, 255, 0), thickness=4, circle_radius=2
        )

    @timeit
    def detect(self, image):
        """
        Detect hands in the given image.

        Args:
            image (np.ndarray): The input image.

        Returns:
            Tuple: A tuple containing the landmarks, bounding boxes, results, and dynamic landmark list.
        """
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = self.hands.process(image)
        image.flags.writeable = True
        bboxes, landmarks, dy_landmark_list = [], [], []

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                landmark_list = []
                # Get the bounding box of the hand
                for landmark in hand_landmarks.landmark:
                    landmark_x = min(
                        int(landmark.x * image.shape[1]), image.shape[1] - 1
                    )
                    landmark_y = min(
                        int(landmark.y * image.shape[0]), image.shape[0] - 1
                    )
                    landmark_list.append([landmark_x, landmark_y])
                dy_landmark_list = landmark_list
                landmarks.append(landmark_list)
                bboxes.append(self.calc_bounding_rect(image, hand_landmarks))

        return landmarks, bboxes, results, dy_landmark_list

    def calc_bounding_rect(self, image, landmarks):
        """
        Calculate the bounding rectangle for a set of landmarks.

        Args:
            image (np.ndarray): The input image.
            landmarks (List): List of landmark coordinates.

        Returns:
            List: Bounding rectangle coordinates [x1, y1, x2, y2].
        """
        image_width, image_height = image.shape[1], image.shape[0]
        landmark_array = np.array(
            [
                [int(landmark.x * image_width), int(landmark.y * image_height)]
                for landmark in landmarks.landmark
            ]
        )
        x, y, w, h = cv.boundingRect(landmark_array)

        return [x, y, x + w, y + h]

    def draw_bounding_rect(self, image, bboxes):
        """
        Draw bounding rectangles on the image.

        Args:
            image (np.ndarray): The input image.
            bboxes (List): List of bounding box coordinates.
        """
        for bbox in bboxes:
            x1, y1, x2, y2 = bbox
            cv.rectangle(image, (x1, y1), (x2, y2), (128, 128, 0), 2)

    def draw_landmarks(self, image, results):
        """
        Draw landmarks on the image.

        Args:
            image (np.ndarray): The input image.
            results: Results from hand detection.

        Returns:
            List: List of bounding box coordinates.
        """
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.landmark_drawing_spec,
                    self.connection_drawing_spec,
                )
