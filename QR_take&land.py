#import libraries
from dronekit import *
import time
import cv2

#capture video
cap=cv2.VideoCapture(0)
detector=cv2.QRCodeDetector()

#connect to vehicle
vehicle=connect('127.0.0.1:14551', baud=921600, wait_ready=True)

#takeoff function
def arm_takeoff(height):
    #check if drone is ready
    while not vehicle.is_armable:
        print("waiting for drone")
        time.sleep(1)

    #change mode to GUIDED and arm
    print("arming")
    vehicle.mode=VehicleMode('GUIDED')
    vehicle.armed=True

    #check if drone is armed
    while not vehicle.armed:
        print("waiting for arm")
        time.sleep(1)

    #takeoff
    print("takeoff")
    vehicle.simple_takeoff(height)

    #report altitude
    while True:
        print('Reached ',vehicle.location.
        global_relative_frame.alt)
        if(vehicle.location.
        global_relative_frame.alt
        >=height*0.95):
            print("reached target")
            break
        time.sleep(1)

while True:
    ret,img=cap.read()
    res,bbox,image2=detector.detectAndDecode(img)
    if res=='1':
        arm_takeoff(10)
        cv2.imshow('video1',image2)
    
    elif res=='2':
        print("land")
        vehicle.mode=VehicleMode("RTL")
        #time.sleep(20)
    
    if ret:
        cv2.imshow('video',img)
    if cv2.waitKey(1)==ord("q"):
        break

#close the connection
vehicle.close()