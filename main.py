from dronekit import *

#Connect to Drone
vehicle = connect('127.0.0.1:14551',baud=921600, wait_ready=True)
print(vehicle.mode)#Returns the status of the drone
print(vehicle.is_armable)#Checks if armable
print(vehicle.armed)#Returns if armed
