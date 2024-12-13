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
import threading


def sigmoid(error, max_speed, agressivity):
    """
    Sigmoid function for smooth speed correction handeling
    :param error: Error for nonlinear correction output
    :param max_speed: The max and min correction value that's reached
    :param agressivity: How aggresive is the easing, the lower the value the lower the easing agressivity (-4.0 to 4.0)
    """
    return 2 * max_speed / (1 + exp(-error * exp(agressivity))) - max_speed


def linear_ease(error, max_speed, min_speed, easing_threshold):
    """
    Eases the speed based on the error, decreasing it smoothly near the threshold.
    Handles both positive and negative errors.
    :param error: The current error value (e.g., distance or angle error, can be negative).
    :param max_speed: The maximum speed when the error is far from the threshold.
    :param min_speed: The minimum speed when the error is close to the threshold.
    :param easing_threshold: The error value below which easing starts.
    :return: Adjusted speed, retaining the sign of the error.
    """
    abs_error = abs(error)  # Ensure we work with the magnitude for calculations

    if abs_error >= easing_threshold:
        # If the absolute error is larger than the threshold, use the maximum speed
        speed = max_speed
    else:
        # Scale speed proportionally based on how close the error is to the threshold
        scaling_factor = abs_error / easing_threshold
        speed = max(min_speed, scaling_factor * max_speed)

    # Retain the direction of the error (negative error => negative speed)
    return speed if error >= 0 else -speed


class Robot:
    def __init__(self, button_start, drift_fix, halal: bool):
        # Initialize motors and sensors
        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.B)
        self.belt_motor = Motor(Port.C)
        
        self.button = TouchSensor(Port.S1)
        # self.us_sensor = UltrasonicSensor(Port.S3)
        
        self.ev3_brick = EV3Brick()
        self.speaker = self.ev3_brick.speaker

        self.default_speed = 200 # mm/s
        self.min_correction_speed = 30 # mm/s
        self.default_turn_speed = 1000
        self.min_turn_correction_speed = 20 
        self.wanted_angle = 0 # deg
        self.time_accuracy = 15

        self.wheel_diameter = 56 # mm
        self.axle_track = 120 # mm
        self.ball_dist = 280 # mm
        self.ball_switch = True # for sorting balls
        self.belt_speed = 1600

        self.drive_base = DriveBase(self.left_motor, self.right_motor, self.wheel_diameter, self.axle_track)
        self.drive_base.settings(self.default_speed, straight_acceleration=self.default_speed / 2, 
                                 turn_rate=self.default_turn_speed, turn_acceleration=self.default_turn_speed * 2)

        print(self.ev3_brick.battery.voltage() / 1000, "V")
        # wait until button is pressed if desired
        while (not self.button.pressed()) and button_start:
            wait(10)

        if halal: self.say("Beesmillaahhh")
        # Drift fix
        if drift_fix == 2:
            while True:
                try:
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
                except Exception as e:
                    continue

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

        if halal: self.say("AArcham du dilahh")
        while (not self.button.pressed()) and button_start:
            wait(10)
        wait(500)

    def say(self, text):
        self.speaker.say(text)
    

    def turn(self, turn_angle, one_wheel=False, aggresivity=-4):
        """
        Turn the robot to the constant angle coordinates using non-linear deceleration.
        :param turn_angle: Angle to add to the current desired angle (in degrees).
        :param aggresivity: Non-linear scaling factor for the correction.
        """

        # Update the desired angle
        self.wanted_angle += turn_angle

        acceleration_factor = 0  # Start with no speed
        max_acceleration_step = 0.025 if not one_wheel else 0.25 * 1.2 # Gradual increase in acceleration factor
        max_acceleration_factor = 1  # Full speed scaling limit

        correct_error_count = 0
        while True:
            current_angle = self.gyro.angle()
            current_angle = (current_angle + 180) % 360 - 180 # normalize the angle
            error = self.wanted_angle - current_angle
            

            # Calculate correction based on the sigmoid function
            correction = sigmoid(error, self.default_turn_speed, aggresivity)
            if abs(correction) < self.min_correction_speed and correction != 0:
                correction = correction / abs(correction) * self.min_correction_speed

            # Gradually increase the acceleration factor
            acceleration_factor = min(
                acceleration_factor + max_acceleration_step, max_acceleration_factor
            )
            correction = correction * acceleration_factor
            
            if one_wheel:
                if turn_angle > 0:  # Clockwise turn
                    self.left_motor.run(abs(correction))
                    self.right_motor.stop()
                else:  # Counterclockwise turn
                    self.right_motor.run(abs(correction))
                    self.left_motor.stop()

            else: 
                self.left_motor.run(correction)
                self.right_motor.run(-correction)

            if abs(error) < 1:  # Threshold for stopping
                correct_error_count += 1

            # finish the correction if accurate in several time checks
            if correct_error_count > self.time_accuracy:
                break

            wait(10)  # Short delay for smooth operation

        self.stop()


    def drive(self, drive_distance, is_close_callback=0):
        """
        Drives the robot a specific distance with gyro stabilization and smooth acceleration.
        :param drive_distance: Distance to drive (in desired units, e.g., mm or cm).
        :param kp_gyro: Proportional gain for gyro correction.
        :param aggressivity: Non-linear scaling factor for the sigmoid function.
        """
        self.drive_base.reset()

        correct_error_count = 0
        acceleration_factor = 0  # Start with no speed
        max_acceleration_step = 0.02  # Gradual increase in acceleration factor
        max_acceleration_factor = 1  # Full speed scaling limit
        kp_gyro = 3
        callback_happened = False

        while True:
            current_distance = self.drive_base.distance()
            distance_error = (drive_distance - current_distance) # Proportionality constant

            # Stop if within the acceptable range
            if abs(distance_error) <= 5:
                correct_error_count += 1

            # Finish the correction if accurate in several time checks
            if correct_error_count > self.time_accuracy:
                break

            # Calculate forward speed using sigmoid for distance error
            target_speed = sigmoid(distance_error, self.default_speed, -4)

            # Gradually increase the acceleration factor
            acceleration_factor = min(
                acceleration_factor + max_acceleration_step, max_acceleration_factor
            )
            forward_speed = target_speed * acceleration_factor

            # Gyro correction for rotational alignment
            angle_error = self.gyro.angle() - self.wanted_angle
            correction = kp_gyro * angle_error

            # Scale down the correction when the robot is moving slower
            correction *= abs(forward_speed / self.default_speed)

            # Flip the correction orientation when going backwards
            correction = (
                -correction * (abs(forward_speed) / forward_speed)
                if forward_speed != 0
                else 0
            )

            if forward_speed <= self.min_correction_speed and not callback_happened and is_close_callback:
                callback_happened = True
                is_close_callback()

            # Drive the robot with calculated forward speed and correction
            self.drive_base.drive(forward_speed, correction)

            wait(10)

        # Stop the robot at the final position
        self.drive_base.stop()

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()


    def pick_ball(self, wait=False):
        belt_motor_angle = self.belt_motor.angle()

        if self.ball_switch:
            self.belt_motor.run_angle(self.belt_speed, 1230, wait=wait) # -700

        else:
            self.belt_motor.run_angle(self.belt_speed, -belt_motor_angle, wait=wait)

        self.ball_switch = not self.ball_switch

    def async_pick_ball(self):
        """
        Operates the belt system asynchronously in a separate thread.
        """
        def pick_task():
            self.pick_ball(wait=True)  # Perform the belt operation
        thread = threading.Thread(target=pick_task)
        thread.start()

    def go_pick(self, ds=0, use_callback=True):
        if use_callback:
            self.drive(self.ball_dist + ds, is_close_callback=self.async_pick_ball)

        else:
            self.drive(self.ball_dist + ds)
            self.async_pick_ball()

        

    def line_collect(self):
        self.go_pick(-100, use_callback=False)
        self.go_pick()
        self.default_speed = 200
        self.go_pick()
        self.default_speed = 200


robot = Robot(button_start=1, drift_fix=1, halal=False)

# this picks up the first ball and moves the belt to correct start pos

robot.pick_ball(wait=True)

robot.turn(-90, one_wheel=True)

robot.line_collect()

robot.turn(90, True)
robot.go_pick(-160)
robot.drive(-10)
robot.turn(90, True)

robot.line_collect()

robot.turn(-90, True)
robot.go_pick(-160)
robot.drive(20)
robot.turn(-90, True)

robot.line_collect()

# empty to small box
robot.drive(-1 * (robot.ball_dist * 3 - 100)) 
wait(3000)
