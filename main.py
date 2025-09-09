import asyncio
import cv2
import threading
import numpy as np
from simple_pid import PID
import serial
import time
import struct


detection_cords = []
display_frame:cv2.typing.MatLike = np.zeros((300, 200))
closest_target_coords = None 
frame = None
frame_center = None
frame_width, frame_height = 0, 0

# Initialize PID controllers
pid_x = PID(Kp=0.0, Ki=0.0, Kd=0.0, setpoint=0)  
pid_y = PID(Kp=0.0, Ki=0.0, Kd=0.0, setpoint=0)
pid_x.output_limits = (-100, 100)  
pid_y.output_limits = (-100, 100)

# Store PID output values
pid_output_x = 0
pid_output_y = 0

#HSV color ranges
lower_red1 = np.array([0, 100, 10])
upper_red1 = np.array([10, 255, 255])

lower_red2 = np.array([160, 75, 10])
upper_red2 = np.array([180, 255, 255])

ser = serial.Serial(port='COM8',  baudrate=921600, timeout=.1, write_timeout=0)
cap = None

laser = 0 #PLACE HOLDER

is_running = True #Used to controll event loop


def init_control_trackbars():
    cv2.namedWindow("main window")
    cv2.resizeWindow("main window", 1000, 565)
    
    cv2.createTrackbar('Lower H1', 'main window', 0, 180, read_trackbars)
    cv2.createTrackbar('Upper H1', 'main window', 10, 180, lambda _ :None)
    cv2.createTrackbar('Lower S1', 'main window', 50, 255, lambda _ :None)
    cv2.createTrackbar('Upper S1', 'main window', 255, 255, lambda _ :None)
    cv2.createTrackbar('Lower V1', 'main window', 49, 255, lambda _ :None)
    cv2.createTrackbar('Upper V1', 'main window', 255, 255, lambda _ :None)

    cv2.createTrackbar('Lower H2', 'main window', 160, 180, lambda _ :None)
    cv2.createTrackbar('Upper H2', 'main window', 180, 180, lambda _ :None)
    cv2.createTrackbar('Lower S2', 'main window', 50, 255, lambda _ :None)
    cv2.createTrackbar('Upper S2', 'main window', 255, 255, lambda _ :None)
    cv2.createTrackbar('Lower V2', 'main window', 14, 255, lambda _ :None)
    cv2.createTrackbar('Upper V2', 'main window', 255, 255, lambda _ :None)

    cv2.createTrackbar("kp",'main window', 20, 500, lambda _ :None)
    cv2.createTrackbar("ki",'main window', 5, 500, lambda _ :None)
    cv2.createTrackbar("kd",'main window', 0, 500, lambda _ :None)


def read_trackbars():
    global lower_red1, upper_red1, lower_red2, upper_red2, pid_x, pid_y

    lower_red1 = np.array([
        cv2.getTrackbarPos("Lower H1", "main window"),
        cv2.getTrackbarPos("Lower S1", "main window"),
        cv2.getTrackbarPos("Lower V1", "main window")
    ])
    upper_red1 = np.array([
        cv2.getTrackbarPos("Upper H1", "main window"),
        cv2.getTrackbarPos("Upper S1", "main window"),
        cv2.getTrackbarPos("Upper V1", "main window")
    ])

    lower_red2 = np.array([
        cv2.getTrackbarPos("Lower H2", "main window"),
        cv2.getTrackbarPos("Lower S2", "main window"),
        cv2.getTrackbarPos("Lower V2", "main window")
    ])
    upper_red2 = np.array([
        cv2.getTrackbarPos("Upper H2", "main window"),
        cv2.getTrackbarPos("Upper S2", "main window"),
        cv2.getTrackbarPos("Upper V2", "main window")
    ])

    
    pid_vals = (
        cv2.getTrackbarPos("kp", "main window")/100.0,
        cv2.getTrackbarPos("ki", "main window")/100.0,
        cv2.getTrackbarPos("kd", "main window")/1000.0
    )
    pid_x.tunings = pid_vals
    pid_y.tunings = pid_vals


def get_pid_values():
    global closest_target_coords, pid_output_x, pid_output_y, frame_height, frame_width
    
    while is_running:
        if closest_target_coords is not None:
            if closest_target_coords == (frame_width//2, frame_height//2):
                pid_output_x = 0
                pid_output_y = 0
            else:
                # Calculate error from center
                x_error = closest_target_coords[0] - frame_width/2
                y_error = closest_target_coords[1] - frame_height/2
                
                # Update PID controllers
                pid_output_x = pid_x(x_error)
                pid_output_y = pid_y(y_error)   
        

def write_to_serial():
    while is_running:
        dx, dy = 0, 0
        x_multiplier, y_multiplier = 1, 1 #for keeping v positive when changing directions

        if pid_output_x>0:
            dx=0
        else:
            dx=1
            x_multiplier=-1

        if pid_output_y>0:
            dy=1
        else:
            dy=0
            y_multiplier=-1

        packet = struct.pack(
            "BBBBBBB",
            0xFF,
            dx,
            dy, 
            int(pid_output_x)*x_multiplier, 
            int(pid_output_y)*y_multiplier,
            laser,
            0xEE
            )
        ser.write(packet)


def detect_red():
    '''
    Detects all red objects and updates "closest_target_coords"
    '''
    global detection_cords, display_frame, closest_target_coords, pid_output_x, pid_output_y, frame_center, frame_width, frame_height, frame, is_running
    
    cap = cv2.VideoCapture(3, cv2.CAP_DSHOW)
    init_control_trackbars()
    read_trackbars()

    threading.Thread(target=get_pid_values, daemon=True).start()
    threading.Thread(target=write_to_serial, daemon=True).start()

    while is_running:
        ret, frame = cap.read()
        read_trackbars()


        frame_height, frame_width = frame.shape[:2]    
        frame_center = (frame_width // 2, frame_height // 2)
        
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)

        # Create masks for both ranges
        mask1 = cv2.inRange(frame_hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(frame_hsv, lower_red2, upper_red2)
        
        # Combine the masks
        filtered_frame = cv2.add(mask1, mask2)

        #Morphologic opening
        filtered_frame = cv2.morphologyEx(filtered_frame, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            filtered_frame,
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
            )
        
        detections = []
        closest_target = None
        min_distance = float('inf')
        
        for contour in contours:
            if cv2.contourArea(contour) > 250:  #filter small contours
                x, y, w, h = cv2.boundingRect(contour)
                detections.append([x,y,x+w,y+h])
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                #Check if closest
                center_x = x + w//2
                center_y = y + h//2
                
                # Calculate distance from frame center to target center
                distance = np.sqrt((center_x - frame_center[0])**2 + (center_y - frame_center[1])**2)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_target = (center_x, center_y)
            
        closest_target_coords = closest_target
        display_frame = frame
        
        if detections == []:
            closest_target_coords = frame_center
            pid_output_x, pid_output_y = 0, 0
        


        #Draw + on frame
        cv2.line(display_frame,
                (frame_center[0]-8, frame_center[1]), 
                (frame_center[0]+8, frame_center[1]), 
                (200, 0, 0),
                2)
        cv2.line(display_frame,
                (frame_center[0], frame_center[1]-8), 
                (frame_center[0], frame_center[1]+8), 
                (200, 0, 0),
                2)

        cv2.imshow('image', display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            cap.release()
            is_running = False
            


if __name__ == "__main__":
    try:
        detect_red()
        print("\nFinished execution...")
    except KeyboardInterrupt:
        print("\nForce exit...")
