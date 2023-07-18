import cv2 as cv
import socket
from hand_detection import HandDetector
from hand_classification import HandClassifier
from utils import CvFpsCalc
from draw import Draw
from process_cmd import GestureCommandProcessor
from collections import deque, Counter
import numpy as np


# Connection settings
def send_Message(message, UDP_IP, UDP_PORT):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.sendto(message.encode('utf8'), (UDP_IP, UDP_PORT))
    udp_socket.close()

def main():
    # Create instances
    hand_detector = HandDetector()
    hand_classifier = HandClassifier()
    cmd_process = GestureCommandProcessor()
    draw = Draw()

    cap = cv.VideoCapture(0)  # Video capture initialization
    cvFpsCalc = CvFpsCalc(buffer_len=10)  # FPS calculation

    # Initialize other variables and settings
    point_history = deque(maxlen=16)
    finger_gesture_history = deque(maxlen=16)

    prev_mode = None  # Initialize previous mode variable

    while True:
        fps = cvFpsCalc.get()  # FPS calculation
        ret, frame = cap.read()  # Read frame

        if not ret:
            break

        # Hand detection
        blank_image = np.zeros(frame.shape, dtype=np.uint8)
        #blank_image = np.full(frame.shape, (128, 128, 128), dtype=np.uint8)
        landmarks, bboxes, results, dy_landmark_list = hand_detector.detect(frame)
        hand_detector.draw_bounding_rect(blank_image, bboxes)
        hand_detector.draw_landmarks(blank_image, results)  # Draw landmarks

        cmd, score, current = None, None, None
        
        # Hand classification
        if landmarks:
            hand_sign_id, confidence_score = hand_classifier.classify(landmarks, frame)
            mode = cmd_process.switch_mode(hand_sign_id)
            current_mode = cmd_process.get_current_mode()
            command = cmd_process.execute_command(hand_sign_id)
            cmd = command

            # Dynamic gesture recognition
            if current_mode == "Formation Control":
                if hand_sign_id in [16, 2]:
                    point_history.append(dy_landmark_list[8])
                else:
                    point_history.append([0, 0])

                # Finger gesture classification
                finger_gesture_id = hand_classifier.dynamic_classify(point_history, frame, len(point_history))
                if finger_gesture_id:
                    finger_gesture_history.append(finger_gesture_id)
                    most_common_fg_id = Counter(finger_gesture_history).most_common(1)
                    gesture_label = hand_classifier.point_history_classifier_labels[most_common_fg_id[0][0]]
                    cmd = gesture_label
                else:
                    cmd = command

            score, current, prev_mode = confidence_score, current_mode, current_mode
            
            # Send message to the control platform
            send_Message(cmd, UDP_IP="127.0.0.1", UDP_PORT=5000)
        else:
            point_history.append([0, 0])
            current = prev_mode
            
        # Draw on the frame #TODO
        draw.real_time_score(blank_image, bboxes, cmd, score) 
        draw.show_fps(blank_image, fps)
        draw.gesture_UI(blank_image, current, cmd)
        # draw.swarm_UI(frame, hand_sign_id, hand_sign_id, hand_sign_id) #TODO: add swarm UI
        
        # Display results or perform actions based on the classification
        cv.imshow("Hand Gesture Based Interactive UAVs Control (HGI) Platform", blank_image)
        key = cv.waitKey(1)
        if key == 27:  # ESC key
            break

    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
