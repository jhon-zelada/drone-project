from modules import drone
drone.connect()
drone.arm_and_takeoff(10)
drone.land()