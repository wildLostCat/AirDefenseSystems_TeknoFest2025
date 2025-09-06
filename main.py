import asyncio
import cv2
import threading
import numpy as np
from simple_pid import PID
import serial

detection_cords = []
display_frame:cv2.typing.MatLike = np.zeros((300, 200))
closest_target_coords = None  
frame_width = 0  # Set in main_loop()
frame_height = 0  

# Initialize PID controllers
pid_x = PID(Kp=0.75, Ki=0.50, Kd=0.02, setpoint=0)  
pid_y = PID(Kp=0.75, Ki=0.55, Kd=0.02, setpoint=0)
pid_x.output_limits = (-500, 500)  
pid_y.output_limits = (-100, 100)

# Store PID output values
pid_output_x = 0
pid_output_y = 0

ser = serial.Serial(port='COM8',  baudrate=115200, timeout=.1, write_timeout=0)

laser = 0 #PLACE HOLDER

is_running = True #Used to controll event loop

def get_pid_values():
    global closest_target_coords, pid_output_x, pid_output_y
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
        
        

async def write_to_serial():
    print("serial coroutine started", flush=True)
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

        # json_string = '{"vx":'+str(int(pid_output_x))+',"vy":'+str(int(pid_output_y))+'}'
        json_string = f'{{"dx":{dx}, "dy":{dy}, "vx":{int(pid_output_x)*x_multiplier}, "vy":{int(pid_output_y)*y_multiplier}, "laser":{laser}}}'

        ser.write(bytes(json_string, 'UTF-8'))
        print(bytes(json_string, 'UTF-8'))

        await asyncio.sleep(.001)


def detect_red(frame:cv2.typing.MatLike):
    '''
    Detects all red objects and updates "closest_target_coords"
    '''
    global detection_cords, display_frame, closest_target_coords, pid_output_x, pid_output_y
    frame_center = (frame_width // 2, frame_height // 2)

    # First range of red (0-10)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    
    # Second range of red (160-180)
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
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
        if cv2.contourArea(contour) > 500:  #filter small contours
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
            
    


async def main_loop():
    global detection_cords, frame_width, frame_height, is_running

    cap = cv2.VideoCapture(0)

    while 1:
        ret, frame = cap.read()
        frame_height, frame_width = frame.shape[:2]
        
        
        threading.Thread(target=detect_red, args=(frame,), daemon=True).start()
        threading.Thread(target=get_pid_values, daemon=True).start()

        cv2.imshow('main window', display_frame)

        if ser.in_waiting>0:
            print(ser.readline().decode(errors="ignore"))
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            cap.release()
            is_running=False
            return
        
        await asyncio.sleep(0.001)


async def main():    
    # Create tasks for concurrent execution and run them
    tasks = [
        main_loop(),
        write_to_serial()
    ]

    try:
        await asyncio.gather(*tasks)
    except Exception as e:
        print(f"Error: {e}", flush=True)



if __name__ == "__main__":
    try:
        asyncio.run(main())
        print("Finished execution...")
    except KeyboardInterrupt:
        print("\nForce exit...")
