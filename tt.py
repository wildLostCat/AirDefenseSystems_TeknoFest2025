import asyncio
import cv2
import numpy as np
from simple_pid import PID
import math
import threading
import serial
import time 
# from sort import Sort 

ser = serial.Serial(port='COM8',  baudrate=115200, timeout=.1,write_timeout=0)
lazer=0
mesafe=0
pidHesapla=1
merkezeDonFlag=0

# track =Sort()

nearest_target_cords = []
frame_average = 1

def test():
    global lazer    
    lazer=1     
    time.sleep(1)
    lazer=0
    
   
    
def nothing(x):
    pass
def write_Serial(hizx,hizy):     
    yonx=0
    yony=0
    if (hizx)>0:           
        yonx=0
    else:        
        hizx=hizx*-1
        yonx=1
    if (hizy)>0:        
        yony=0
    else:
        
        hizy=hizy*-1
        yony=1

        
    json_string = '{"lazer":'+str(lazer)+',"yonx":'+str(yonx)+',"yony":'+str(yony)+' ,"hizx": '+str(int(hizx))+',"hizy": '+str(int(hizy))+'}'        
    ser.write(bytes(json_string,  'utf-8'))
    print(bytes(json_string,  'utf-8'))

# Pencere oluştur ve trackbar ekle


cv2.namedWindow('Trackbars')
cv2.namedWindow('pid')
cv2.resizeWindow('pid', 400, 300)

cv2.createTrackbar('Lower H1', 'Trackbars', 0, 180, nothing)
cv2.createTrackbar('Upper H1', 'Trackbars', 10, 180, nothing)
cv2.createTrackbar('Lower S1', 'Trackbars', 100, 255, nothing)
cv2.createTrackbar('Upper S1', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('Lower V1', 'Trackbars', 49, 255, nothing)
cv2.createTrackbar('Upper V1', 'Trackbars', 255, 255, nothing)

cv2.createTrackbar('Lower H2', 'Trackbars', 160, 180, nothing)
cv2.createTrackbar('Upper H2', 'Trackbars', 180, 180, nothing)
cv2.createTrackbar('Lower S2', 'Trackbars', 75, 255, nothing)
cv2.createTrackbar('Upper S2', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('Lower V2', 'Trackbars', 14, 255, nothing)
cv2.createTrackbar('Upper V2', 'Trackbars', 255, 255, nothing)

cv2.createTrackbar("kp",'pid', 29, 500, nothing)
cv2.createTrackbar("ki",'pid', 82, 500, nothing)
cv2.createTrackbar("kd",'pid', 16, 500, nothing)


def average_target_cords():
    count = len(nearest_target_cords)
    if count > frame_average:
        nearest_target_cords.pop(0)
    x = [i[0] for i in nearest_target_cords] # Get all X's
    y = [i[1] for i in nearest_target_cords]

    return (sum(x)/count, sum(y)/count)


def distance(x1,x2,y1,y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def hesapla(x,y,w,h):
    global mesafe
    if (mesafe==0):                
        mesafe=distance((x+x+w)/2,center_x,(y+y+h)/2,center_y)
        takipEdilenKordinat[0]=(x+x+w)/2
        takipEdilenKordinat[1]=(y+y+h)/2
    else:
        if(mesafe>distance((x+x+w)/2,center_x,(y+y+h)/2,center_y)):
            mesafe=distance((x+x+w)/2,center_x,(y+y+h)/2,center_y)
            takipEdilenKordinat[0]=(x+x+w)/2
            takipEdilenKordinat[1]=(y+y+h)/2
        #   kp   ki  kd  0.2, 0.5, 0
pidx = PID(0.1, 0.1 ,0.1, setpoint=0)
pidx.output_limits=(-10,10)

pidy = PID(0.1, 0.1, 0.1, setpoint=0)
pidy.output_limits=(-10,10)
nearistIndex=0
mesafe=0
takipEdilenKordinat=[320,240]

# tracker = Sort()

# VideoCapture nesnesi oluştur
cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Reduced frame size for faster processing
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    line = ser.readline()
    # print(line)
    # Kameradan bir kare oku
    ret, frame = cap.read()
    height, width, _ = frame.shape
    center_x, center_y = width // 2-4, (height // 2 -29)
    
    # BGR'den HSV'ye çevir
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    # Trackbar değerlerini oku
    lower_h1 = cv2.getTrackbarPos('Lower H1', 'Trackbars')
    lower_s1 = cv2.getTrackbarPos('Lower S1', 'Trackbars')
    lower_v1 = cv2.getTrackbarPos('Lower V1', 'Trackbars')
    upper_h1 = cv2.getTrackbarPos('Upper H1', 'Trackbars')
    upper_s1 = cv2.getTrackbarPos('Upper S1', 'Trackbars')
    upper_v1 = cv2.getTrackbarPos('Upper V1', 'Trackbars')
    
    lower_h2 = cv2.getTrackbarPos('Lower H2', 'Trackbars')
    lower_s2 = cv2.getTrackbarPos('Lower S2', 'Trackbars')
    lower_v2 = cv2.getTrackbarPos('Lower V2', 'Trackbars')
    upper_h2 = cv2.getTrackbarPos('Upper H2', 'Trackbars')
    upper_s2 = cv2.getTrackbarPos('Upper S2', 'Trackbars')
    upper_v2 = cv2.getTrackbarPos('Upper V2', 'Trackbars')

    kp = cv2.getTrackbarPos('kp', 'pid') / 100.0
    ki = cv2.getTrackbarPos('ki', 'pid') / 100.0
    kd = cv2.getTrackbarPos('kd', 'pid') / 1000.0


    pidx.tunings = (kp, ki, kd)
    pidy.tunings = (kp, ki, kd)


    
    #frame = cv2.medianBlur(frame, 5)
    #frame = cv2.medianBlur(frame, 5)
    #frame = cv2.medianBlur(frame, 5)
    #frame = cv2.medianBlur(frame, 5)
    #frame = cv2.GaussianBlur(frame,(13,13),11)
    # Kırmızı renk aralıklarını tanımla
    lower_red1 = np.array([lower_h1, lower_s1, lower_v1])
    upper_red1 = np.array([upper_h1, upper_s1, upper_v1])
    Trackbars1 = cv2.inRange(hsv, lower_red1, upper_red1)
  
    
    lower_red2 = np.array([lower_h2, lower_s2, lower_v2])
    upper_red2 = np.array([upper_h2, upper_s2, upper_v2])
    Trackbars2 = cv2.inRange(hsv, lower_red2, upper_red2)
    
    # İki Trackbarseyi birleştir
    Trackbars = Trackbars1 + Trackbars2
    
    # Morfolojik açma işlemi uygula
    
    Trackbars = cv2.morphologyEx(Trackbars, cv2.MORPH_OPEN, kernel)
    
    # Bulanıklaştırma işlemi uygula


    # Trackbarseyi uygula
    res = cv2.bitwise_and(frame, frame, mask=Trackbars)
    
    # Konturları bul ve filtrele
    contours, _ = cv2.findContours(Trackbars, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detections = []
    mesafe=0
    for contour in contours:
        if cv2.contourArea(contour) > 1000:  # Küçük konturları filtrele
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,255,0),2)
            thread = threading.Thread(target=hesapla, args=(x,y,w,h)).start()

            detections.append([x,y,x+w,y+h])  


    if  detections: 
        # tracks = tracker.update(np.array(detections))
        nearest = None
        min_distance = 1e9
        for(x1,y1,x2,y2,t_id) in tracks.astype(int): 
            cx_t = (x1 + x2) // 2
            cy_t = (y1 + y2) // 2
            d = distance(cx_t, center_x, cy_t, center_y)
            if d < min_distance:
                min_distance = d
                
                
                nearest_target_cords.append((cx_t,cy_t))
                nearest = average_target_cords()
                

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"ID: {t_id}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if nearest:
            takipEdilenKordinat[0] = nearest[0]
            takipEdilenKordinat[1] = nearest[1]

    if  len(contours) == 0:
            takipEdilenKordinat[0] = center_x
            takipEdilenKordinat[1] = center_y
            threading.Thread(target = write_Serial,args=(0,0),daemon=True).start()

    if distance(takipEdilenKordinat[0], center_x, takipEdilenKordinat[1], center_y) < 2:
            threading.Thread(target = test,daemon=True).start()

    vx = pidx(takipEdilenKordinat[0] - center_x)       
    vy = pidy(takipEdilenKordinat[1] - center_y)       
    threading.Thread(target=write_Serial, args=(vx, vy), daemon=True).start()





    # if(len(contours)==0):
    #     merkezeDonFlag+=1
    #     thread = threading.Thread(target=write_Serial, args=(1,1))
    #     thread.start() 
    #     pidHesapla=0
    # else:
    #     merkezeDonFlag=0
    #     pidHesapla=1
    
    # if merkezeDonFlag>60:
    #     pass
    #     thread = threading.Thread(target=write_Serial, args=(1,1))
    #     thread.start() 

#------------------------------------------------------------------------------------------------------
    # if(len(contours)==0):
    #     thread = threading.Thread(target=write_Serial, args=(0,0))
    #     thread.start()               
    # if(distance(takipEdilenKordinat[0],center_x,takipEdilenKordinat[1],center_y)<2):
    #    thread = threading.Thread(target=test)
    #    thread.start() 
   
               
    # control = pidx(takipEdilenKordinat[0]-center_x)
    # control2 = pidy(takipEdilenKordinat[1]-center_y)   
    # thread = threading.Thread(target=write_Serial, args=(control,control2))
    # thread.start()  
    # if control!=0 or control2!=0:
    #     thread = threading.Thread(target=write_Serial, args=(control,control2))
    #     thread.start() 
#------------------------------------------------------------------------------------------------------
    # MOTPY ile takip et
   
    cv2.line(frame, ( int(takipEdilenKordinat[0]) - 5, int(takipEdilenKordinat[1])), (int(takipEdilenKordinat[0]) +5, int(takipEdilenKordinat[1])), (0, 255, 255), 2)
    cv2.line(frame, (int(takipEdilenKordinat[0]), int(takipEdilenKordinat[1]) - 5), (int(takipEdilenKordinat[0]), int(takipEdilenKordinat[1]) + 5), (0, 255, 255), 2)
    cv2.line(frame, (center_x - 20, center_y), (center_x +20, center_y), (255, 0, 0), 2)
    cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (255, 0, 0), 2)

    
    
    # Sonuçları göster
    cv2.imshow('Frame', frame)
    cv2.imshow('Trackbars', Trackbars)
    cv2.imshow('Trackbars', Trackbars)
    cv2.imshow('res', res)
    
    # 'q' tuşuna basıldığında döngüyü kır
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Her şeyi kapat
cap.release()
cv2.destroyAllWindows()