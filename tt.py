import struct
import serial
import time

ser = serial.Serial("COM8", 921600, timeout=.1, write_timeout=0)
p_time = time.time()
counter=0

try:
    x, y = 5, 250
    while True:

        data = struct.pack(
            "BBBB",
            0xFF,
            x,
            y,
            0xFC
        )
        ser.write(data)
        # counter+=1
        # if (time.time()-p_time)>=1.0:
        #     p_time=time.time()
        #     print(counter)
        #     counter=0


        x+=1
        y-=1
        if x == 250:
            x=0
        if y == 5:
            y=250
        # time.sleep(4/10000)
        print(f"x:{x}, y:{y}")
        if ser.in_waiting:
            print(f"ser: {ser.readline().decode(errors="ignore")}")
except KeyboardInterrupt:
    print("\nquit")
    quit()


