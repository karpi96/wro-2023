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
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

motor = Motor(Port.D)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
# Initialize the color sensor.
line_sensor = ColorSensor(Port.S1)
timer = StopWatch()
timer2 = StopWatch()
# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=108)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 3
WHITE = 80
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = -100
"""
class Position:
    def __init__(this):
        this.x= 0
        this.y = 0

position = Position()

position.x+=1
"""
countX = 0
countY = -1
count = 0
# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2

a = True
b= True

def pickUp():
    motor.run_angle(550, 1900, wait=True)


def putDown():
    motor.run_angle(550, -1900, wait=True)


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


def DockScan():
    robot.straight(100)
    robot.straight(-200)
    robot.straight(100)
def goToDock():
    robot.straight(-80)
    turnLeft()
    wait(100)
    straightleft()
    robot.straight(-2500)
    turnLeft()
    DockScan()
    turnRight()
    count+= 1


while True:
    straightleft()
    wait(30)
    if (a == True and line_sensor.reflection() < 10):
        robot.straight(-70)
        turnRight()
        robot.turn(10)
        wait(30)
        straightleft()
        robot.straight(-200)
        straightleft()
        a = False
        turnLeft()

        robot.straight(-400)
        
        if line_sensor.reflection() > 30:
            turnRight()
            robot.turn(10)
            motor.run_angle(550, -300, wait=True)
            robot.straight(150)
            motor.run_angle(550, 300, wait=True)
            robot.straight(-150)
        
            turnRight()
            robot.straight(-270)
            if b==True and line_sensor.reflection()>40 and line_sensor.reflection()<55:
                robot.straight(-80)
                turnLeft()
                b=False
                while count<2:
                    straightleft()
                    if line_sensor.reflection()<10:
                        count+1
    if count==2:
        turnLeft()
        robot.straight(-400)
        break



            
""""""
""""
while True:
    straightleft()
    wait(30)
    if (a==True and line_sensor.reflection() < 10):
        robot.straight(-70)
        turnRight()
        wait(30)
        a=False
    if line_sensor.reflection() < 10 and timer.time() > 2000:
        count=count+1
        timer.reset()
    if count==3:
        wait(30)
        robot.straight(-50)
        turnLeft()
        wait(30)
        robot.straight(-400)
        break
"""""
"""""
while True:
    straightleft()
    wait(30)
    if (a == True and line_sensor.reflection() < 10):
        robot.straight(-70)
        turnRight()
        robot.turn(10)
        wait(30)
        straightleft()
        robot.straight(-410)
        straightleft()
        a = False
        
    if (count == 0 and line_sensor.reflection() < 10):
        goToDock()
    while count < 2 and line_sensor.reflection() > 10:
        straightleft()
        if line_sensor.reflection() < 10:
            count +=0.5
    if count == 2:
        fullTurn()
        straightleft()
    if count == 2 and line_sensor.reflection() < 10:
        robot.straight(-70)
        turnRight()
        count += 0.5
        while count == 2.5:
            straightleft()
            if line_sensor.reflection() < 10:
                count += 0.5
    if count == 3 and line_sensor.reflection() < 10:
        goToDock()
    while count < 5 and line_sensor.reflection() > 10:
        straightleft()
        if line_sensor.reflection() < 10:
            count = count+0.5
    if count == 5:
        fullTurn()
        straightleft()
    while count == 5 and line_sensor.reflection() > 10:
        straightleft()
        if line_sensor.reflection() < 10:
            count += 1
    if count == 6:
        break
"""""

"""""
    if count==1 and line_sensor.reflection() < 10 and timer.time() > 2000:
        turnLeft()
        count=count+1
        robot.straight(-700)
        fullTurn()
"""""

# red,green,blue = f ()
"""
while True:
    putDown()
    wait(200)
    pickUp()
    wait(200)
"""
