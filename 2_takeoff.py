from dronekit import *
import time
vehicle=connect('127.0.0.1:14551',baud=921600,wait_ready=True)

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


def arm_takeoff(height):
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
    else:
        print("Vehicle is not armed")
    print("takeoff")
    vehicle.simple_takeoff(height)
    while True:
        print("Reached: ",vehicle.location.global_relative_frame.alt)
        if(vehicle.location.global_relative_frame.alt>=height*0.98):
            print("reached")
            break
        time.sleep(1)
arm_takeoff(100)
print('Hovering')
time.sleep(10)
#arm_takeoff(10)
print("Changing Velocity")
send_ned_velocity(5,0,0,10)
time.sleep(3)
print("Changing Velocity")
send_ned_velocity(0,5,0,10)
time.sleep(3)
print("Changing Velocity")
send_ned_velocity(0,0,5,10)
time.sleep(3)
print("Changing Velocity")
send_ned_velocity(0,0,-10,10)
time.sleep(3)
print("Changing Velocity")
send_ned_velocity(-5,0,0,10)
time.sleep(3)
print("Changing Velocity")
send_ned_velocity(0,-5,0,10)
time.sleep(3)
print("Changing Velocity")
print("preparing to decend")
vehicle.mode=VehicleMode("RTL")
#print(vehicle.location.global_relative_frame.alt)
while vehicle.location.global_relative_frame.alt>=0.5:
    print("landing...", vehicle.location.global_relative_frame.alt)
    time.sleep(1)
print("resting")
time.sleep(5)
vehicle.armed=False
while vehicle.armed:
    print("diasming...")
print("closing communtication with drone")
vehicle.close()