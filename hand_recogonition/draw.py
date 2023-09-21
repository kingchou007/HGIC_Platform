import cv2 as cv
import numpy as np

class Draw():
    def draw_text(self, image, text, position, color):
        cv.putText(image, text, position, cv.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv.LINE_AA)

    def real_time_score(self, image, bboxes, command, hand_sign_score):
        if command is not None and bboxes:
            hand_sign_score = round(hand_sign_score * 100, 2)
            # create the information content
            info_text = f"CMD: {command}, {hand_sign_score}%"
            
            x1, y1, _, _ = bboxes[0]  # Get the coordinates of the first bounding box %one hand
            
            # Display the information text on the image
            if x1 >= 0 and y1 >= 0:
                cv.putText(image, info_text, (x1 + 5, y1 - 10), 
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv.LINE_AA)
    
    def gesture_UI(self, image, mode, command):
        x = 30  # Left-aligned
        y_mode = 90 # Distance from the top for mode text
        y_command = 120  # Distance from the top for command text
        
        # Display the mode text
        cv.putText(image, f"Current Mode: {mode}", (x, y_mode), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv.LINE_AA)
        # cv.putText(image, f"Current Mode: {mode}", (x, y_mode), 
        #            cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
        
        # Display the command text
        cv.putText(image, f"Current Command: {command}", (x, y_command), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv.LINE_AA)
        # cv.putText(image, f"Current Command: {command}", (x, y_command), 
        #            cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)
    
    def human_UI(self, image):
        # Define box's position and size
        top_left = (5, 10)
        bottom_right = (295, 250)
        title = "Human"
        # cv.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
        # title_position = (top_left[0], top_left[1]-10)
        # cv.putText(image, title, title_position, cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv.rectangle(image, top_left, bottom_right, (0, 0, 0), 2)
        text_position = (30, 50)
        # cv.putText(image, title, text_position, cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
        # cv.putText(image, title, text_position, cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        cv.putText(image, title, text_position, cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5, cv.LINE_AA)
        cv.putText(image, title, text_position, cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)

    
    def swarm_info(self, image):
        x = 30  # Left-aligned
        y_1 = 340 # Distance from the top for mode text
        y_2 = 370  # Distance from the top for command text
        y_3 = 400
        
        temp = "9"
        # Display the mode text
        cv.putText(image, f"Swarm Size: {temp}", (x, y_1), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv.LINE_AA)
        # cv.putText(image, f"Current Mode: {mode}", (x, y_mode), 
        #            cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
        
        # Display the command text
        cv.putText(image, f"Collisions: {'0'}", (x, y_2), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv.LINE_AA)
        # cv.putText(image, f"Current Command: {command}", (x, y_command), 
        #            cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)
        
        cv.putText(image, f"Current Status: {temp}", (x, y_3), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv.LINE_AA)
        # cv.putText(image, f"Current Command: {command}", (x, y_command), 
        #            cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)

    def robot_UI(self, image):
        # Define box's position and size
        top_left = (5, 260)
        bottom_right = (295, 715)
        title = "Swarm Info"
        # cv.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
        # title_position = (top_left[0], top_left[1]-10)
        # cv.putText(image, title, title_position, cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv.rectangle(image, top_left, bottom_right, (0, 0, 0), 2)
        text_position = (30, 300)
        # cv.putText(image, title, text_position, cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
        # cv.putText(image, title, text_position, cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        cv.putText(image, title, text_position, cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5, cv.LINE_AA)
        cv.putText(image, title, text_position, cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)
        

    def show_fps(self, image, fps):
        text = "FPS: " + str(fps) # create the information content
        text_size, _ = cv.getTextSize(text, cv.FONT_HERSHEY_SIMPLEX, 1.0, 4) # get the text size
        text_width = text_size[0] # get text width
        x = (image.shape[1] - text_width) // 2 + 150 # calculate the x-coordinate of the text
        y = 50 # calculate the y-coordinate of the text

        # Display the information text on the image
        cv.putText(image, text, (x, y), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv.LINE_AA)
        cv.putText(image, text, (x, y), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)
    