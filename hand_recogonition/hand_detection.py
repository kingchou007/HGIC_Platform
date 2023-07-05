import cv2 as cv
import mediapipe as mp
import numpy as np

class HandDetector:
    def __init__(
        self,
        use_static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    ):
        
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=use_static_image_mode,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self.mp_drawing = mp.solutions.drawing_utils    # hand detection
        
        # visualizing the hand landmarks and connections on the image    
        self.landmark_drawing_spec = self.mp_drawing.DrawingSpec(color=(255, 0, 0), 
                                                                 thickness=3, 
                                                                 circle_radius=2)
        
        self.connection_drawing_spec = self.mp_drawing.DrawingSpec(color=(0, 255, 0), 
                                                                   thickness=3, 
                                                                   circle_radius=2) 
    
    def detect(self, image):
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        image.flags.writeable = False        # image is no longer writeable to improve performance
        results = self.hands.process(image)  # process the image
        image.flags.writeable = True         # image is now writeable again

        landmarks = []  # a list of landmarks for the detected hands
        bboxes = []     # a list of bounding boxes for the detected hands
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                landmark_list = []
                
                # Get the bounding box of the hand
                for landmark in hand_landmarks.landmark:
                    landmark_x = min(int(landmark.x * image.shape[1]), image.shape[1] - 1)
                    landmark_y = min(int(landmark.y * image.shape[0]), image.shape[0] - 1)
                    landmark_list.append([landmark_x, landmark_y])
                landmarks.append(landmark_list)
                bboxes.append(self.calc_bounding_rect(image, hand_landmarks))
                # brect = calc_bounding_rect(debug_image, hand_landmarks)

        return landmarks, bboxes, results
    
    def calc_bounding_rect(self, image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]
        landmark_array = np.array([[int(landmark.x * image_width), 
                                    int(landmark.y * image_height)] for landmark in landmarks.landmark])
        x, y, w, h = cv.boundingRect(landmark_array)
        
        return [x, y, x + w, y + h]
            
    def draw_bounding_rect(self, image, bboxes):
        for bbox in bboxes:
            x1, y1, x2, y2 = bbox
            cv.rectangle(image, (x1, y1), (x2, y2), (128,128,0), 2) # 0, 255, 0 = green
            
    def draw_landmarks(self, image, results):
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    image, 
                    hand_landmarks, 
                    self.mp_hands.HAND_CONNECTIONS, # draw the connections between the landmarks
                    self.landmark_drawing_spec,     # landmark drawing spec  
                    self.connection_drawing_spec    # connection drawing spec
                )
    

