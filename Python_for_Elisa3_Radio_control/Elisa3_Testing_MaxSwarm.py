import elisa3
import time

robotAddr = [4050, 4096, 4469, 4021, 4060,
             4104, 4101, 4083, 3991, 3988,
             3948, 4061, 4111, 4086, 4105,
             4472, 4471, 4054, 4095, 4090,
             4098, 4114, 4128, 4123, 4034,
             4001, 4005, 4028, 4035]
elisa = elisa3.Elisa3(robotAddr)
elisa.start()

counter = 0
turn_duration = 2  # This needs calibration, the time required to turn 180 degrees
speed = 10  # Speed for moving forward

while True:
    for addr in robotAddr:
        if counter == 0:
            # Move forward
            elisa.setLeftSpeed(addr, speed)
            elisa.setRightSpeed(addr, speed)
        elif counter == 10:
            # Stop
            elisa.setLeftSpeed(addr, 0)
            elisa.setRightSpeed(addr, 0)
        elif counter == 20:
            # Start turning 180 degrees - begin turn
            elisa.setLeftSpeed(addr, speed)
            elisa.setRightSpeed(addr, -speed)
        elif counter == 20 + turn_duration:
            # Finish turning 180 degrees - stop turn
            elisa.setLeftSpeed(addr, 0)
            elisa.setRightSpeed(addr, 0)
            counter = -1  # Reset counter to -1 because it will be incremented to 0 at the end of the loop

    counter += 1
    time.sleep(0.1)  # Adjust timing as necessary