import csv
import copy
import itertools
from model import KeyPointClassifier, PointHistoryClassifier

class HandClassifier:
    def __init__(self, 
                 static_label_path = './model/static/keypoint_classifier_label.csv', 
                 dynamic_label_path = './model/dynamic/point_history_classifier_label.csv'
        ):
        self.keypoint_classifier = KeyPointClassifier()
        self.point_history_classifier = PointHistoryClassifier()
        self.keypoint_classifier_labels = self.load_labels(static_label_path)
        self.point_history_classifier_labels = self.load_labels(dynamic_label_path)
        
    def classify(self, landmarks, image):
        if landmarks:
            landmark_list = landmarks[0]  # Assuming only one hand
            pre_processed_landmark_list = self.pre_process_landmark(landmark_list)
            hand_sign_id, confidence_score = self.keypoint_classifier(pre_processed_landmark_list)
            return hand_sign_id, confidence_score

        return None, None

    def dynamic_classify(self, point_history, image):
        point_history_ids = []

        if point_history:
            pre_processed_point_history_list = self.pre_process_point_history(image, point_history)
            point_history_id = self.point_history_classifier(pre_processed_point_history_list)
            point_history_ids.append(point_history_id)

        return point_history_ids
    
    def load_labels(self, file_path):
        with open(file_path, encoding='utf-8-sig') as f:
            labels = csv.reader(f)
            labels = [row[0] for row in labels]
        return labels

    def pre_process_landmark(self, landmark_list):
        temp_landmark_list = copy.deepcopy(landmark_list)

        base_x, base_y = 0, 0
        for index, landmark_point in enumerate(temp_landmark_list):
            if index == 0:
                base_x, base_y = landmark_point[0], landmark_point[1]

            temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
            temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y

        temp_landmark_list = list(itertools.chain.from_iterable(temp_landmark_list))

        max_value = max(list(map(abs, temp_landmark_list)))

        def normalize_(n):
            return n / max_value

        temp_landmark_list = list(map(normalize_, temp_landmark_list))

        return temp_landmark_list

    def pre_process_point_history(self, image, point_history):
        image_width, image_height = image.shape[1], image.shape[0]

        temp_point_history = copy.deepcopy(point_history)

        base_x, base_y = 0, 0
        for index, point in enumerate(temp_point_history):
            if index == 0:
                base_x, base_y = point[0], point[1]

            temp_point_history[index][0] = (temp_point_history[index][0] - base_x) / image_width
            temp_point_history[index][1] = (temp_point_history[index][1] - base_y) / image_height

        temp_point_history = list(itertools.chain.from_iterable(temp_point_history))

        return temp_point_history
