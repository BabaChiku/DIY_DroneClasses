# TechVidvan hand Gesture Recognizer

# import necessary packages

import cv2
import numpy as np
import mediapipe as mp
import tensorflow as tf
from tensorflow.keras.models import load_model
from dronekit import *
import time
import threading

# initialize mediapipe
mpHands = mp.solutions.hands
hands = mpHands.Hands(max_num_hands=1, min_detection_confidence=0.5)
mpDraw = mp.solutions.drawing_utils

#connect to Drone
vehicle=connect('127.0.0.1:14551',baud=921600,wait_ready=True)

# Load the gesture recognizer model
model = load_model('mp_hand_gesture')

# Load class names
f = open('gesture.names', 'r')
classNames = f.read().split('\n')
f.close()
print(classNames)


# Initialize the webcam
cap = cv2.VideoCapture(0)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def armIt():
    if not(vehicle.armed):
        while not vehicle.is_armable:
            print("waiting for drone")
            time.sleep(1)
        print("arming")
        vehicle.mode=VehicleMode('GUIDED')
        vehicle.armed=True
        while not vehicle.armed:
                print("Drone arming...")
                time.sleep(1)
        if(vehicle.armed):
            print("Vehicle is armed")
    
def disarmIt():
    if vehicle.armed:
        vehicle.armed=False
        while vehicle.armed:
            print("diasming...")
        print('SAFE')

def in_flight():
    if vehicle.location.global_relative_frame.alt<0.5 and vehicle.location.global_relative_frame.alt>-0.5:
        print("Flight on land")
        return False
    else:
        print("Flight in air")
        return True

def getHeight():
    if in_flight():
        send_ned_velocity(0,0,-5,1)
    else:
        vehicle.simple_takeoff(50)
        while True:
            print("climbing: ",vehicle.location.global_relative_frame.alt)
            if(vehicle.location.global_relative_frame.alt>=50*0.99):
                print("reached")
                break
            time.sleep(1)

def getBack():
    vehicle.mode=VehicleMode("RTL")
    while in_flight():
        print("returning to home"+str(in_flight()))
        time.sleep(1)

def loseHeight():
    send_ned_velocity(0,0,2,1)

'''
def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;

def get_distance_metres(aLocation1, aLocation2):

    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):

    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print("Reached target\n")
            break;
        time.sleep(2)
'''

def moveForward():
    if in_flight():
        send_ned_velocity(5,0,0,1)

def moveBackward():
    if in_flight():
        send_ned_velocity(-5,0,0,1)

def moveRight():
    if in_flight():
        send_ned_velocity(0,5,0,1)

def moveLeft():
    if in_flight():
        send_ned_velocity(0,-5,0,1)
    
def sendSignal(className):
    if className=='fist':
                armIt()
    elif className=='stop':
        if in_flight():
            print("Disaming a risk")
            #getBack()
        else:
            disarmIt()
    elif className=='thumbs up':
        print("Asscend")
        getHeight()
    elif className=='thumbs down':
        loseHeight()
    elif className=='call me':
        getBack()
    elif className=='peace':
        moveForward()
    elif className=='rock':
        moveBackward()
    elif className=='okay':
        moveRight()
    elif className=='smile':
        moveLeft()
    elif className=='live long':
        pass
    print("Thread complete"+className)

t = threading.Thread(target=sendSignal, args=('live long',))
count = 0
while True:
    # Read each frame from the webcam
    _, frame = cap.read()

    x, y, c = frame.shape

    # Flip the frame vertically
    frame = cv2.flip(frame, 1)
    framergb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Get hand landmark prediction
    result = hands.process(framergb)

    # print(result)
    
    className = ''

    # post process the result
    if result.multi_hand_landmarks:
        landmarks = []
        for handslms in result.multi_hand_landmarks:
            for lm in handslms.landmark:
                # print(id, lm)
                lmx = int(lm.x * x)
                lmy = int(lm.y * y)

                landmarks.append([lmx, lmy])

            # Drawing landmarks on frames
            mpDraw.draw_landmarks(frame, handslms, mpHands.HAND_CONNECTIONS)

            # Predict gesture
            prediction = model.predict([landmarks])
            # print(prediction)
            classID = np.argmax(prediction)
            className = classNames[classID]
            if not t.is_alive():
                print("In thread")
                if(className=="call me" and count>=5):
                    t = threading.Thread(target=sendSignal, args=(className,))
                    t.start()
                    count=0
                elif(className=="call me"):
                    count+=1
                else:
                    t = threading.Thread(target=sendSignal, args=(className,))
                    t.start()

    # show the prediction on the frame
    cv2.putText(frame, className, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
    cv2.putText(frame, str(count), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)

    # Show the final output
    cv2.imshow("Output", frame) 

    if cv2.waitKey(1) == ord('q'):
        break

# release the webcam and destroy all active windows
cap.release()

cv2.destroyAllWindows()