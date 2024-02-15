import elisa3
import time

robotAddr = [
3890,4471,4111,4098,4068,4054,4114,
4472,4001,4086,4128,4035,4123,4105,
4028,4005,4034,3846,4010,4049,4031,
4095,4090,4061,3918,3869,3828,3868,
3904,3901,3981,4124,3829,4020,4019,
4047,3887,3926,3823,3948,4060,4021,
4469,3988,4104,4050,4096,4101,4083
]
print('Total robots: ',len(robotAddr))
elisa = elisa3.Elisa3(robotAddr)
elisa.start()

counter = 0
turn_duration = 2  # This needs calibration, the time required to turn 180 degrees
speed = 0  # Speed for moving forward

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