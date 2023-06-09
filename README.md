# PyMAVLink Drone Control

This repository provides an implementation of pyMAVLink functions to control a drone using the MAVLink protocol. MAVLink is a lightweight messaging protocol that here is used to communicate a flight controller with a companion computer in this case being a raspberry.

  
## Features

- Arm and disarm the drone

- Takeoff and land the drone

- Control drone movement (e.g., yaw, pitch, roll, throttle)

- Monitor drone status and telemetry information


## Prerequisites

- Python 3.6 or higher

-  `pymavlink` library: Install using `pip3 install pymavlink`
- `tqdm` library to monitor progress : Install using `pip3 install tqdm`