from pymavlink import mavutil
import time
from pymavlink.quaternion import QuaternionBase
import math
from tqdm import tqdm
drone = None
LAND_VELOCITY=0.5
def connect(connection_string='udpin:localhost:14551'):
    global drone
    if drone == None:
        drone =mavutil.mavlink_connection(connection_string)
    drone.wait_heartbeat() 
    print("Heartbeat from system (system %u component %u)" % 
      (drone.target_system, drone.target_component))

def return_to_launch():
    global drone
    print("Return to launch mode")
    drone.mav.command_long_send(drone.target_system,
                                     drone.target_component, 
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                                     0, 
                                     1,
                                     6, # value for RTL mode
                                     0,0,0,0,0)
    
def land():
    global drone
    drone.mav.command_long_send(drone.target_system,
                                     drone.target_component, 
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                                     0, 
                                     1,
                                     9,# 9 value for land mode
                                     0,0,0,0,0)
    
    # Wait for landing completion
    prev_position=None
    stable_count=0
    estimated_landing_time = abs(drone.recv_match(type='LOCAL_POSITION_NED', blocking=True).z)/LAND_VELOCITY+3  # in seconds

    with tqdm(total=100, unit='%', desc='Landing', bar_format='{l_bar}{bar}| {n_fmt}/{total_fmt}') as pbar:
        start_time = time.time()

        while True:
            msg = drone.recv_match(type=['GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED'], blocking=True)
            current_time = time.time()
            elapsed_time = current_time - start_time

            if msg is not None:
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    current_position = (msg.relative_alt / 1000.0, )  # Convert to meters
                else:  # 'LOCAL_POSITION_NED'
                    current_position = (msg.z, )  # NED coordinates in meters

                if prev_position is not None:
                    if abs(current_position[0] - prev_position[0]) < 0.1:  # Adjust the threshold as needed
                        stable_count += 1
                    else:
                        stable_count = 0

                prev_position = current_position

                if stable_count >= 10 and elapsed_time >= estimated_landing_time :  # Adjust the count as needed
                    pbar.update(100- pbar.n)
                    break
            progress = min(100, int((elapsed_time / estimated_landing_time) * 100))  # Calculate progress based on elapsed time
            pbar.update(progress - pbar.n)  # Increment the progress bar

            time.sleep(0.1)  # Update the progress bar every 0.1 seconds

    print("Landing completed successfully.")


def set_to_auto():
    global drone
    print("Auto mode enabled")
    drone.mav.command_long_send(drone.target_system,
                                     drone.target_component, 
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                                     0, 
                                     1,
                                     3, # 3 value for auto mode
                                     0,0,0,0,0)
def is_armable():
    global drone
    drone.param_fetch_all()
    # Wait for the response
    while True:
        msg = drone.recv_match(type='PARAM_VALUE')
        if msg is not None and msg.param_id == 'ARMING_CHECK':
            armable=bool(msg.param_value) 
            return armable # Returns response if drone is armable

def is_armed():
    global drone
    while True:
        msg = drone.recv_match(type='HEARTBEAT', blocking=True)
        if msg is not None:
            armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            return armed    

def arm_and_takeoff(aTargetAltitude):
    global drone
    print ("Check if drone is armable")
    # Don't try to arm until autopilot is ready
    
    while not is_armable():
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)
    print ("Arming motors")
    # Copter should arm in GUIDED mode
    drone.mav.command_long_send(drone.target_system,drone.target_component, 
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1,4,0,0,0,0,0)
    #drone.armed   = True
    drone.arducopter_arm()

    # Confirm vehicle armed before attempting to take off
    while not is_armed():
        print (" Waiting for arming...")
        time.sleep(1)
        
    print ("Taking off!")
    drone.mav.command_long_send(drone.target_system,drone.target_component, 
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0,aTargetAltitude)

    c_alt=0 # Current altitude
    with tqdm(total=100, unit='%', desc='Ascending', bar_format='{l_bar}{bar}| {n_fmt}/{total_fmt}') as pbar:
        while c_alt>aTargetAltitude*-0.99:
            c_alt= float(drone.recv_match(type='LOCAL_POSITION_NED',blocking=True).z)
            progress = min(100, int(abs(c_alt/ aTargetAltitude) * 100))  # Calculate progress based on elapsed time
            pbar.update(progress - pbar.n)  # Increment the progress bar
            time.sleep(0.1)
        pbar.update(100-pbar.n)
    
    print("Reached target altitude")
    print("ALtitude: ", abs(c_alt))
    time.sleep(1) # a little time to hover before executing the next instruction 
    #  after Vehicle.simple_takeoff will execute immediately).


def movement_YAW_command(heading):
    global drone
    speed = 0 
    direction = 1 #direction -1 ccw, 1 cw
    
    #heading 0 to 360 degree. if negative then ccw 
    
    print("Sending YAW movement command with heading: %f" % heading)

    if heading < 0:
        heading = heading*-1
        direction = -1

    #point drone into correct heading 
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,       
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0,          
        heading,    
        speed,      #speed deg/s
        direction,  
        1,          #relative offset 1
        0, 0, 0)    

def movement_attitude(duration, roll,pitch,yaw,thrust=0.5):
    global drone
    start_time = time.time()
    while time.time() - start_time < duration:
        drone.mav.set_attitude_target_send(
            0,                      # Time since boot in milliseconds
            1,                      # Target system ID
            1,                      # Target component ID
            0b00000111,             # Type mask: Ignore all body rate fields
            QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]), 
            0,                      # Target body roll rate (not used)
            0,                      # Target body pitch rate (not used)
            0,                      # Target body yaw rate (no change in yaw angle)
            thrust                  # Thrust value for backward movement (positive value)
        )
        time.sleep(0.1)  # Adjust the time interval between commands if needed

def movement_vx_vy_command(velx, vely, altitude):
    global drone

    #velocity_x positive = forward. negative = backwards
    #velocity_y positive = right. negative = left
    #velocity_z positive = down. negative = up (Yes really!)

    print("Sending XYZ movement command with v_x(forward/backward): %f v_y(right/left): %f " % (velx,vely))
    drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,drone.target_system, 
        drone.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED,
        int(0b110111000011),
        0,0,altitude,
        velx,vely,0,
        0,0,0,
        0,0))