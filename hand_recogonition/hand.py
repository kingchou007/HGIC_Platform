import cv2 as cv
import socket
from hand_detection import HandDetector
from hand_classification import HandClassifier
from utils import CvFpsCalc
from draw import Draw
from process_cmd import GestureCommandProcessor
from collections import deque, Counter
import numpy as np
import time
from timingdecorator.timeit import timeit

# Sends a UDP message
@timeit
def send_Message(message, UDP_IP, UDP_PORT):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.sendto(message.encode('utf8'), (UDP_IP, UDP_PORT))
    udp_socket.close()

# Adds a sidebar to an image
def add_sidebar(image, sidebar_width, sidebar_color):
    sidebar = np.full((image.shape[0], sidebar_width, 3), sidebar_color, dtype=np.uint8)
    return np.concatenate((sidebar, image), axis=1)

def main():
    # Initialization
    hand_detector = HandDetector()
    hand_classifier = HandClassifier()
    cmd_process = GestureCommandProcessor()
    draw = Draw()
    cap = cv.VideoCapture(0)  
    cvFpsCalc = CvFpsCalc(buffer_len=10)

    # Variables for gesture tracking
    point_history = deque(maxlen=16)
    finger_gesture_history = deque(maxlen=16)
    prev_mode = None

    while True:
        fps = cvFpsCalc.get()  
        ret, frame = cap.read()  
        frame = add_sidebar(frame, 300, (224, 230, 241))
        if not ret:
            break
        
        # Hand detection and drawing
        landmarks, bboxes, results, dy_landmark_list = hand_detector.detect(frame)
        hand_detector.draw_bounding_rect(frame, bboxes)
        hand_detector.draw_landmarks(frame, results)

        cmd, score, current = None, None, None

        # Hand gesture classification
        if landmarks:
            hand_sign_id, confidence_score = hand_classifier.classify(landmarks, frame)
            mode = cmd_process.switch_mode(hand_sign_id)
            current_mode = cmd_process.get_current_mode()
            command = cmd_process.execute_command(hand_sign_id)
            cmd = command

            # Dynamic gesture processing
            if current_mode == "Formation":
                point_history.append(dy_landmark_list[8] if hand_sign_id in [16, 2] else [0, 0])
                
                # Finger gesture classification
                finger_gesture_id = hand_classifier.dynamic_classify(point_history, frame, len(point_history))
                if finger_gesture_id:
                    finger_gesture_history.append(finger_gesture_id)
                    most_common_fg_id = Counter(finger_gesture_history).most_common(1)
                    cmd = hand_classifier.point_history_classifier_labels[most_common_fg_id[0][0]]
                else:
                    cmd = command

            score, current, prev_mode = confidence_score, current_mode, current_mode

            # Send the identified command over UDP
            send_Message(cmd, UDP_IP="127.0.0.1", UDP_PORT=5000)
        else:
            point_history.append([0, 0])
            current = prev_mode
        
        # Rendering visual feedback on the frame
        draw.real_time_score(frame, bboxes, cmd, score) 
        draw.show_fps(frame, fps)
        draw.gesture_UI(frame, current, cmd)
        draw.human_UI(frame)
        draw.robot_UI(frame)
        draw.swarm_info(frame)

        # Display the processed frame
        cv.imshow("Hand Gesture Based Interactive UAVs Control (HGI) Platform", frame)
        key = cv.waitKey(1)
        if key == 27:  # Exit if ESC is pressed
            break

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()