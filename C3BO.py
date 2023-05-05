#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
main_motor = Motor(Port.D)
# Initialize the color sensor.
line_sensor = ColorSensor(Port.S1)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=108)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2
# Start following the line endlessly.
sideRight = True
def straightSideRight():
    PROPORTIONAL_GAIN = -1.2
    deviation = line_sensor.reflection() - threshold

    turn_rate = PROPORTIONAL_GAIN * deviation

    robot.drive(DRIVE_SPEED, turn_rate)

    wait(10)

def straightSideLeft():
    PROPORTIONAL_GAIN = 1.2
    deviation = line_sensor.reflection() - threshold
    turn_rate = PROPORTIONAL_GAIN * deviation
    robot.drive(DRIVE_SPEED, turn_rate)
    wait(10)

def goLeftSide():
    global sideRight
    if sideRight == True:
        sideRight = False
        robot.drive(DRIVE_SPEED,20)
        wait(100)
        robot.drive(DRIVE_SPEED,0)
        wait(50)
def goRightSide():
    global sideRight
    if sideRight == False:
        sideRight = True
        robot.drive(DRIVE_SPEED,20)
        wait(100)
        robot.drive(DRIVE_SPEED,0)
        wait(50)
        

while True:
    count=0
    while count<=5:
        straightSideRight()
        wait(100)
        count=+1
    wait(100)
    goLeftSide()
    wait(100)
    while count<=10:
        straightSideLeft()
        wait(100)
        count=+1
    goRightSide()
    wait(200)
    straightSideRight()
    if count==10:
        break

