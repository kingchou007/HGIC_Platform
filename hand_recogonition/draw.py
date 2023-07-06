import cv2 as cv

class Draw():
    def draw_text(self, image, text, position, color):
        cv.putText(image, text, position, cv.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv.LINE_AA)

    def real_time_score(self, image, bboxes, command, hand_sign_score):
        if command is not None and bboxes:
            hand_sign_score = round(hand_sign_score * 100, 2)
            # create the information content
            info_text = f"ID: {command}, {hand_sign_score}%"
            
            x1, y1, _, _ = bboxes[0]  # Get the coordinates of the first bounding box %one hand
            
            # Display the information text on the image
            if x1 >= 0 and y1 >= 0:
                cv.putText(image, info_text, (x1 + 5, y1 - 10), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)
    
    def gesture_UI(self, image, mode, command):
        x = 10  # Left-aligned
        y_mode = 30  # Distance from the top for mode text
        y_command = 60  # Distance from the top for command text
        
        # Display the mode text
        cv.putText(image, f"Current Mode: {mode}", (x, y_mode), 
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv.LINE_AA)
        cv.putText(image, f"Current Mode: {mode}", (x, y_mode), 
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)
        
        # Display the command text
        cv.putText(image, f"Current Command: {command}", (x, y_command), 
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv.LINE_AA)
        cv.putText(image, f"Current Command: {command}", (x, y_command), 
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)

    def swarm_UI(self, image, swarm_status, current_task, task_progress):
        x = image.shape[1] - 320  # Right-aligned
        y_swarm = 30  # Distance from the top for swarm status text
        y_task = 60  # Distance from the top for current task text
        y_progress = 90  # Distance from the top for task progress text
        
        # Display swarm status text
        cv.putText(image, f"Swarm: {swarm_status}", (x, y_swarm), 
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 4, cv.LINE_AA)
        cv.putText(image, f"Swarm: {swarm_status}", (x, y_swarm), 
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)
        
        # Display current task text
        cv.putText(image, f"Status: {current_task}", (x, y_task), 
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 4, cv.LINE_AA)
        cv.putText(image, f"Status: {current_task}", (x, y_task), 
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)
        
        # Display task progress text
        cv.putText(image, f"Progress: {task_progress}", (x, y_progress), 
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 4, cv.LINE_AA)
        cv.putText(image, f"Progress: {task_progress}", (x, y_progress), 
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)

    def show_fps(self, image, fps):
        text = "FPS: " + str(fps) # create the information content
        text_size, _ = cv.getTextSize(text, cv.FONT_HERSHEY_SIMPLEX, 1.0, 4) # get the text size
        text_width = text_size[0] # get text width
        x = (image.shape[1] - text_width) // 2 # calculate the x-coordinate of the text
        y = 50 # calculate the y-coordinate of the text

        # Display the information text on the image
        cv.putText(image, text, (x, y), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv.LINE_AA)
        cv.putText(image, text, (x, y), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv.LINE_AA)