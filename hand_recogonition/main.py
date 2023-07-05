import cv2 as cv
import socket
from hand_detection import HandDetector
from hand_classification import HandClassifier
from utils import CvFpsCalc
from hand_cmd import GestureCommandProcessor

# Connection settings
def send_Message(message, UDP_IP, UDP_PORT):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.sendto(message.encode(), (UDP_IP, UDP_PORT))
    udp_socket.close()

# Main function
def main():
    # UDP settings
    
    # Create instances of HandDetector and HandClassifier
    hand_detector = HandDetector()
    hand_classifier = HandClassifier()
    
    # Create instance of GestureCommandProcessor
    cmd_process = GestureCommandProcessor()

    # Initialize other variables and settings
    point_history = []

    # Video capture initialization
    cap = cv.VideoCapture(0)
    
    # FPS calculation
    fps = CvFpsCalc()

    while True:
        ret, frame = cap.read()

        if ret:
            # Hand detection
            landmarks, bboxes, results = hand_detector.detect(frame)  # detect hands in the frame
            hand_detector.draw_bounding_rect(frame, bboxes)           # draw bounding box
            hand_detector.draw_landmarks(frame, results)              # draw landmarks
            
            # Hand classification
            hand_sign_id, confidence_score = hand_classifier.classify(landmarks, frame)
            #print(hand_sign_id, confidence_score)
            
            # Update point historyclassify
            # point_history.append(landmarks)
        
            mode = cmd_process.switch_mode(hand_sign_id)
            command = cmd_process.execute_command(hand_sign_id)
            
            # Execute emergency command if the gesture corresponds to an emergency command
 
            # print(command)
            
            
            
            
            
            # Send message to controller
            # if True:
            #     send_Message(hand_sign_id, UDP_IP="127.0.0.1", UDP_PORT=5000)
                
                
        
    
            
            
        
            

            # Display results or perform actions based on the classification

            cv.imshow("Hand Gesture Recognition", frame)
        else:
            break

        key = cv.waitKey(1)
        if key == 27:  # ESC key
            break

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
