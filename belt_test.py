#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import Stop
 
# Initialize the motor and touch sensor
motor = Motor(Port.C)
touch_sensor = TouchSensor(Port.S1)
ev3 = EV3Brick()
 
# Set initial motor state as stopped
motor_running = False
 
while True:
    # Check if the touch sensor is pressed
    if touch_sensor.pressed():
        # Toggle motor state
        if motor_running:
            motor.stop()
            motor_running = False
        else:
            motor.run(500)  # Set the motor to spin at 500 degrees per second
            motor_running = True
       
        # Wait until the sensor is released to avoid multiple triggers
        while touch_sensor.pressed():
            wait(10)  # Small delay to debounce the input
