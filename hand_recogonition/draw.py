import cv2 as cv
import numpy as np

class Draw():
    def draw_text(self, image, text, position, color):
        """Draws specified text on the image at the given position with the provided color."""
        cv.putText(image, text, position, cv.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv.LINE_AA)

    def real_time_score(self, image, bboxes, command, hand_sign_score):
        """Displays the command and its confidence score on the image."""
        if command is not None and bboxes:
            hand_sign_score = round(hand_sign_score * 100, 2)
            info_text = f"CMD: {command}, {hand_sign_score}%"
            x1, y1, _, _ = bboxes[0]
            if x1 >= 0 and y1 >= 0:
                cv.putText(image, info_text, (x1 + 5, y1 - 10), 
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv.LINE_AA)
    
    def gesture_UI(self, image, mode, command):
        """Displays the current mode and command on the image."""
        x, y_mode, y_command = 30, 90, 120
        cv.putText(image, f"Current Mode: {mode}", (x, y_mode), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv.LINE_AA)
        cv.putText(image, f"Current Command: {command}", (x, y_command), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv.LINE_AA)
    
    def human_UI(self, image):
        """Draws the Human UI section on the image."""
        top_left, bottom_right, title = (5, 10), (295, 250), "Human"
        text_position = (30, 50)
        cv.rectangle(image, top_left, bottom_right, (0, 0, 0), 2)
        cv.putText(image, title, text_position, cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5, cv.LINE_AA)
        cv.putText(image, title, text_position, cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)

    def swarm_info(self, image):
        """Displays the swarm-related information on the image."""
        x, y_1, y_2, y_3 = 30, 340, 370, 400
        temp = "9"
        cv.putText(image, f"Swarm Size: {temp}", (x, y_1), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv.LINE_AA)
        cv.putText(image, f"Collisions: {'0'}", (x, y_2), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv.LINE_AA)
        cv.putText(image, f"Current Status: {temp}", (x, y_3), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv.LINE_AA)

    def robot_UI(self, image):
        """Draws the Robot UI section on the image."""
        top_left, bottom_right, title = (5, 260), (295, 715), "Swarm Info"
        text_position = (30, 300)
        cv.rectangle(image, top_left, bottom_right, (0, 0, 0), 2)
        cv.putText(image, title, text_position, cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5, cv.LINE_AA)
        cv.putText(image, title, text_position, cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)

    def show_fps(self, image, fps):
        """Displays the frames-per-second (FPS) on the image."""
        text = f"FPS: {fps}"
        text_size, _ = cv.getTextSize(text, cv.FONT_HERSHEY_SIMPLEX, 1.0, 4)
        x, y = (image.shape[1] - text_size[0]) // 2 + 150, 50
        cv.putText(image, text, (x, y), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv.LINE_AA)
        cv.putText(image, text, (x, y), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)