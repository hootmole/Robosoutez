#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


import math
 
class Robot:
    def __init__(self):
        # Initialize motors and sensors
        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.B)
        self.gyro = GyroSensor(Port.S2)
        self.us_sensor = UltrasonicSensor(Port.S3)
        self.button = TouchSensor(Port.S1)
        self.belt_motor = Motor(Port.C)
        
        self.ev3_brick = EV3Brick()
        self.speaker = self.ev3_brick.speaker

        self.default_speed = 400
        self.wanted_angle = 0
        self.wheel_diameter = 56
       
        # Reset gyro angle to 0 on initialization
        self.gyro.reset_angle(0)
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)

    def say(self, text):
        self.speaker.say(text)
    
    def turn(self, added_angle, max_speed=100, kp=2.5, kd=0.5):
        self.say("I'm turning" + str(added_angle))
        """
        Turn the robot to the constant angle coords
        :param angle: Angle in degrees (positive for clockwise, negative for counterclockwise)
        :param max_speed: Maximum turning speed in degrees per second
        :param kp: Proportional gain
        :param kd: Derivative gain
        """
        self.wanted_angle += added_angle
        # self.wanted_angle = self.wanted_angle % 360 # mod the wanted angle to not cululate angles
        last_error = 0

        while True:
            print(self.gyro.angle(), self.wanted_angle)
            # Calculate the error (how far we are from the target angle)
            error = self.wanted_angle - self.gyro.angle()
            derivative = error - last_error
            
            # Calculate the control output using PD control
            correction = kp * error + kd * derivative
            correction = max(-max_speed, min(max_speed, correction))  # Clamp to max speed
            
            # Set motor speeds
            self.left_motor.run(correction)
            self.right_motor.run(-correction)
            
            # Exit condition: when the error is small enough
            if abs(error) < 1:  # Tolerance of 1 degree
                break
            
            last_error = error
            wait(10)  # Small delay to avoid overloading the processor

        # Stop motors once the target angle is reached
        self.stop()
       

    def drive_until_obstacle(self, distance_to_object, kp=1.5, correction_distance=50):
        """
        Moves the robot straight toward a barrier and adjusts to the desired distance using ultrasonic feedback.
        :param distance_to_object: Desired distance from the obstacle in mm
        :param kp: Proportional gain for gyro correction
        """

        # Phase 1: Drive toward the obstacle
        while True:
            print(self.gyro.angle(), self.wanted_angle)

            # Break if the target distance is reached
            if self.us_sensor.distance() < distance_to_object + correction_distance:
                print("LETS GO")
                self.left_motor.stop()
                self.right_motor.stop()
                break


            # Gyro correction
            angle_error = self.gyro.angle() - self.wanted_angle
            correction = kp * angle_error
            # print("error: " + str(angle_error) + 
            #     ", c: " + str(correction))

            # Adjust motor speeds based on correction
            self.left_motor.run(self.default_speed - correction)
            self.right_motor.run(self.default_speed + correction)

            wait(10)  # Small delay for smoother operation

        # Phase 2: Fine-tune the distance
        self.say("Just some minor finishing touches")

        while True:
            self.speaker.beep()

            # Measure the current distance to the obstacle
            current_distance = self.us_sensor.distance()
            print("d: " + str(current_distance) + " mm")
            error = current_distance - distance_to_object
            print("de: " + str(error) + " mm")

            # Stop if within the acceptable range
            if abs(error) <= 5:
                break

            # Determine speed for fine-tuning
            speed = max(-200, min(200, kp * error))  # Clamp speed for safety
            print("Fine-tuning speed: " + str(speed))

            self.left_motor.run(speed)
            self.right_motor.run(speed)

            wait(10)

        # Stop the robot at the final position
        self.stop()


    def wait_until_pressed(self):
        while not self.button.pressed():
            wait(10)
        self.say("Alright, I'm going now!")

    def collect(self, run, speed=1000):
        if run:
            self.belt_motor.run(speed)
        else:
            self.belt_motor.stop()


    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
 
 

kratkazed=130

robot = Robot()
robot.wait_until_pressed()
robot.collect(1)
while not robot.button.pressed():
    robot.drive_until_obstacle(300)
    print("going until 300")
    robot.turn(-180)
    print("turning -180")
    robot.drive_until_obstacle(100)
    print("going until 100")
    robot.turn(180)
    print("turning 180")


