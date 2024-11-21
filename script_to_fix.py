#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import ( AnalogSensor)

import math
from math import exp

def sigmoid(error, max_speed, agressivity):
    """
    Sigmoid function for smooth speed correction handeling
    :param error: Error for nonlinear correction output
    :param max_speed: The max and min correction value that's reached
    :param agressivity: How aggresive is the easing, the lower the value the lower the easing agressivity (-4.0 to 4.0)
    """
    return 2 * max_speed / (1 + exp(-error * exp(agressivity))) - max_speed

 
class Robot:
    def __init__(self, button_start, drift_fix):
        # Initialize motors and sensors
        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.B)
        self.belt_motor = Motor(Port.C)
        
        self.button = TouchSensor(Port.S1)
        self.us_sensor = UltrasonicSensor(Port.S3)
        
        self.ev3_brick = EV3Brick()
        self.speaker = self.ev3_brick.speaker

        self.default_speed = 800 # mm/s
        self.min_correction_speed = 30 # mm/s
        self.default_turn_speed = 500
        self.min_turn_correction_speed = 20 
        self.wanted_angle = 0 # deg

        self.wheel_diameter = 52 # mm
        self.axle_track = 120 # mm
        self.us_sensor_to_wheelbase_dist = 100 # mm
        self.belt_tube_dist = 215 # mm
        self.ball_dist = 280 # mm ?
        self.belt_wheel_radius = 20 # mm ?

        self.drive_base = DriveBase(self.left_motor, self.right_motor, self.wheel_diameter, self.axle_track)
        self.drive_base.settings(self.default_speed, straight_acceleration=self.default_speed * 2, 
                                 turn_rate=self.default_turn_speed, turn_acceleration=self.default_turn_speed * 2)

        # wait until button is pressed if desired
        while (not self.button.pressed()) and button_start:
            wait(10)
        self.say("Bismillahhh")
        # Drift fix
        if drift_fix == 2:
            while True:
                self.speaker.beep()
                AnalogSensor(Port.S2)
                self.speaker.beep()
                wait(100)
                self.gyro = GyroSensor(Port.S2)
                self.speaker.beep()
                
                if self.gyro.speed() == 0:
                    break
                self.gyro.reset_angle(0)
                self.speaker.beep()

        if drift_fix == 1:
            self.gyro = GyroSensor(Port.S2)
            while True:
                self.gyro.speed()
                self.gyro.angle()
                self.speaker.beep()
                if self.gyro.speed() == 0:
                    break
                wait(100)
       
        # Reset gyro angle to 0 on initialization
        self.gyro.reset_angle(0)
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)

    def say(self, text):
        self.speaker.say(text)
    
    def turn(self, added_angle, aggresivity=-4.5):
        """
        Turn the robot to the constant angle coordinates using non-linear deceleration.
        :param added_angle: Angle to add to the current desired angle (in degrees).
        :param aggresivity: Non-linear scaling factor for the correction.
        """

        # Update the desired angle
        self.wanted_angle += added_angle
        self.wanted_angle = self.wanted_angle# % 360  # Wrap angle to [0, 360)

        while True:
            current_angle = self.gyro.angle()# % 360  # Wrap current angle to [0, 360)
            error = self.wanted_angle - current_angle

            # Adjust error for shortest turning path, subtract the value to adjust for shortest angle
            # if error > 180: error -= 360
            # elif error < -180: error += 360

            print("gyro:", current_angle, "  wanted:", self.wanted_angle, "  error:", error)

            # Calculate correction based on the sigmoid function
            correction = sigmoid(error, self.default_turn_speed, aggresivity)
            if abs(correction) < self.min_correction_speed and correction != 0:
                correction = correction / abs(correction) * self.min_correction_speed
            
            self.left_motor.run(correction)
            self.right_motor.run(-correction)

            if abs(error) < 1:  # Threshold for stopping
                break

            wait(10)  # Short delay for smooth operation

        self.stop()

       

    def drive_until_obstacle(self, distance_to_object, kp_gyro=1, ki_gyro=0.0, aggressivity=-5):
        """
        Moves the robot straight toward a barrier and adjusts to the desired distance using ultrasonic and gyro feedback.
        :param distance_to_object: Desired distance from the obstacle in mm.
        :param kp_gyro: Proportional gain for gyro correction.
        :param kp_distance: Proportional gain for distance correction.
        :param max_speed: Maximum speed for driving.
        :param aggressivity: Aggressiveness for sigmoid smoothing.
        """

        angle_error_integral = 0 # sum all angle errors to approximate it's integral
        while True:
            current_distance = self.us_sensor.distance()
            # calculate the distance error, adjusted for displacement of the us sensor from wheelbase
            distance_error = current_distance - distance_to_object #  + self.us_sensor_to_wheelbase_dist

            # Stop if within the acceptable range
            if abs(distance_error) <= 5:
                break

            # Calculate forward speed using sigmoid for distance error
            forward_speed = sigmoid(distance_error, self.default_speed, aggressivity)
            if abs(forward_speed) < self.min_correction_speed and forward_speed != 0:
                forward_speed = forward_speed / abs(forward_speed) * self.min_correction_speed

            # Gyro correction for rotational alignment
            angle_error = self.gyro.angle() - self.wanted_angle
            angle_error_integral += angle_error
            correction = kp_gyro * angle_error + ki_gyro * angle_error_integral

            correction = correction * (forward_speed / self.default_speed) # scale down the correction when robot is moving slower
            # self.left_motor.run(forward_speed - correction)
            # self.right_motor.run(forward_speed + correction)
            self.drive_base.drive(forward_speed, -correction)

            wait(10)

        # Stop the robot at the final position
        self.drive_base.stop()


    def drive(self, drive_distance, kp_gyro=1, ki_gyro = 0.0, aggressivity=-6):
        self.drive_base.reset()

        angle_error_integral = 0
        while True:
            current_distance = self.drive_base.distance()
            distance_error = (drive_distance - current_distance) * 17/10 # + self.us_sensor_to_wheelbase_dist (+ proporionality error fix const)

            # Stop if within the acceptable range
            if abs(distance_error) <= 5:
                break

            # Calculate forward speed using sigmoid for distance error
            forward_speed = sigmoid(distance_error, self.default_speed, aggressivity)
            if abs(forward_speed) < self.min_correction_speed and forward_speed != 0:
                forward_speed = forward_speed / abs(forward_speed) * self.min_correction_speed
            
            # Gyro correction for rotational alignment
            angle_error = self.gyro.angle() - self.wanted_angle
            angle_error_integral += angle_error
            correction = kp_gyro * angle_error + ki_gyro * angle_error_integral

            correction = correction * (forward_speed / self.default_speed) # scale down the correction when robot is moving slower
            # self.left_motor.run(forward_speed - correction)
            # self.right_motor.run(forward_speed + correction)
            self.drive_base.drive(forward_speed, -correction)

            wait(10)

        # Stop the robot at the final position
        self.drive_base.stop()

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()


    def collect(self, run):
        """
        Collect balls
        :param run: if you want to start collectiong the balls or not
        :param kp_gyro: Speed of the motor spin in deg/s
        """
        if run:
            # Calculate belt speed
            belt_angular_velocity = (self.default_speed * self.belt_tube_dist) / (self.belt_wheel_radius * self.ball_dist) * (180 / math.pi) # deg/s
            self.belt_motor.run(belt_angular_velocity)
        else:
            self.belt_motor.stop()
    
