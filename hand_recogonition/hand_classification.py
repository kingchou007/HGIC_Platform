import csv
import copy
from model import KeyPointClassifier, PointHistoryClassifier

class HandClassifier:
    def __init__(self, static_label_path='./model/static/keypoint_classifier_label.csv',
                 dynamic_label_path='./model/dynamic/point_history_classifier_label.csv'):
        self.keypoint_classifier = KeyPointClassifier()
        self.point_history_classifier = PointHistoryClassifier()
        self.keypoint_classifier_labels = self.load_labels(static_label_path)
        self.point_history_classifier_labels = self.load_labels(dynamic_label_path)

    def classify(self, landmarks, image):
        """
        Classify a static hand gesture based on landmarks.

        Args:
            landmarks (List): List of hand landmarks.
            image (np.ndarray): The input image.

        Returns:
            Tuple: Hand sign ID and confidence score.
        """
        if landmarks:
            landmark_list = landmarks[0]  # Assuming only one hand
            pre_processed_landmark_list = self.pre_process_landmark(landmark_list)
            hand_sign_id, confidence_score = self.keypoint_classifier(pre_processed_landmark_list)
            return hand_sign_id, confidence_score

        return None, None

    def dynamic_classify(self, point_history, image, history_length=16):
        """
        Classify a dynamic hand gesture based on a short period of recorded gestures.

        Args:
            point_history (List): List of points in the hand's trajectory.
            image (np.ndarray): The input image.
            history_length (int): Length of the point history to consider.

        Returns:
            int: Dynamic gesture ID.
        """
        if point_history:
            dynamic_gesture_id = 0
            pre_processed_point_history_list = self.pre_process_point_history(image, point_history)
            point_history_len = len(pre_processed_point_history_list)

            if point_history_len == (history_length * 2):
                dynamic_gesture_id = self.point_history_classifier(pre_processed_point_history_list)
            return dynamic_gesture_id
        return None

    def process_dynamic_gesture(self, most_common_fg_id):
        """
        Process the most common gesture ID.

        Args:
            most_common_fg_id (List): List of the most common finger gesture IDs.

        Returns:
            str: Command corresponding to the finger gesture.
        """
        cmd = self.point_history_classifier_labels[most_common_fg_id[0][0]]
        return cmd

    def load_labels(self, file_path):
        """
        Load labels from a CSV file, where store the labels for the classifier.

        Args:
            file_path (str): Path to the CSV file.

        Returns:
            List: List of labels.
        """
        with open(file_path, encoding='utf-8-sig') as f:
            labels = csv.reader(f)
            labels = [row[0] for row in labels]
        return labels

    def pre_process_landmark(self, landmark_list):
        """
        Pre-process the landmark list.

        Args:
            landmark_list (List): List of hand landmarks.

        Returns:
            List: Pre-processed landmark list.
        """
        temp_landmark_list = copy.deepcopy(landmark_list)

        base_x, base_y = 0, 0
        for landmark_point in temp_landmark_list:
            if not base_x and not base_y:
                base_x, base_y = landmark_point[0], landmark_point[1]

            landmark_point[0] -= base_x
            landmark_point[1] -= base_y

        max_value = max(abs(value) for landmark_point in temp_landmark_list for value in landmark_point)

        def normalize_(n):
            return n / max_value

        temp_landmark_list = [normalize_(value) for landmark_point in temp_landmark_list for value in landmark_point]

        return temp_landmark_list

    def pre_process_point_history(self, image, point_history):
        """
        Pre-process the point history.

        Args:
            image (np.ndarray): The input image.
            point_history (List): List of points in the hand's trajectory.

        Returns:
            List: Pre-processed point history.
        """
        image_width, image_height = image.shape[1], image.shape[0]

        temp_point_history = copy.deepcopy(point_history)

        base_x, base_y = 0, 0
        for point in temp_point_history:
            if not base_x and not base_y:
                base_x, base_y = point[0], point[1]

            point[0] = (point[0] - base_x) / image_width
            point[1] = (point[1] - base_y) / image_height

        temp_point_history = [value for point in temp_point_history for value in point]

        return temp_point_history
