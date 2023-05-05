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
from pybricks.tools import wait,StopWatch
from pybricks.robotics import DriveBase

motor = Motor(Port.D)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
# Initialize the color sensor.
line_sensor = ColorSensor(Port.S1)
timer=StopWatch()
timer2=StopWatch()
# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=108)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 3
WHITE = 80
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = -100
count=0
# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2

a = True

def pickUp():
    motor.run_angle(550,900,wait=True)

def putDown():
    motor.run_angle(550 ,-1500,wait=True)
def fullTurn():
    robot.turn(170)
def turnRight():
    robot.turn(80)
def turnLeft():
    robot.turn(-80)
def straightleft():
    deviation = line_sensor.reflection() - threshold
    turn_rate = PROPORTIONAL_GAIN * deviation
    robot.drive(DRIVE_SPEED, turn_rate)
    wait(10)
def topDockScan():
    robot.straight(100)
    robot.straight(-200)
    robot.straight(100)
while True:
    putDown()
    wait(200)
    pickUp()
    wait(200)
    pickUp()
    wait(200)
