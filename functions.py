#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait
 
class Robot:
    def __init__(self):
        # Initialize motors and sensors
        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.B)
        self.gyro = GyroSensor(Port.S2)
        self.us_sensor = UltrasonicSensor(Port.S3)
        self.button = TouchSensor(Port.S1)
        self.belt_motor = Motor(Port.C)
       
        # Reset gyro angle to 0 on initialization
        self.gyro.reset_angle(0)
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
       
    def turn(self, angle, speed=100):
        """
        Turns the robot by a specified angle using the gyro sensor.
        :param angle: Angle in degrees (positive for clockwise, negative for counterclockwise)
        :param speed: Turning speed in degrees per second
        """
        target_angle = self.gyro.angle() + angle
        direction = 1 if angle > 0 else -1
 
        # Start turning at the specified speed
        self.left_motor.run(direction * speed)
        self.right_motor.run(-direction * speed)
       
        # Keep turning until the robot reaches the target angle
        while (self.gyro.angle() - target_angle) * direction < 0:
            wait(10)
       
        # Stop motors
        self.stop()
       
    def go_straight(self, distance, speed=200):
        """
        Moves the robot straight for a specified distance.
        :param distance: Distance in degrees of wheel rotation
        :param speed: Speed in degrees per second
        """
        # Reset motor positions
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
       
        # Run both motors to the target position
        self.left_motor.run_target(speed, distance, Stop.COAST, wait=False)
        self.right_motor.run_target(speed, distance, Stop.COAST)
   
    def stop(self):
        """
        Stops both motors immediately.
        """
        self.left_motor.stop(Stop.BRAKE)
        self.right_motor.stop(Stop.BRAKE)
 
    def drive_until_obstacle(self, speed=200, distance=200):
        """
        Moves forward until an obstacle is detected within a certain distance.
        :param speed: Speed in degrees per second
        :param distance: Distance to obstacle in mm
        """
        # Move forward until an obstacle is detected within the specified distance
        self.left_motor.run(speed)
        self.right_motor.run(speed)
       
        while self.us_sensor.distance() > distance:
            wait(10)
       
        # Stop when an obstacle is detected
        self.stop()
 
    def turn_to_angle(self, target_angle, speed=100):
        """
        Turns the robot to face a specific angle (relative to the initial reset angle).
        :param target_angle: Absolute target angle relative to the initial reset
        :param speed: Turning speed in degrees per second
        """
        current_angle = self.gyro.angle()
        angle_to_turn = target_angle - current_angle
        self.turn(angle_to_turn, speed)
 
    def wait_until_pressed(self):
        while not self.button.pressed():
            wait(10)
 
    def collect(self, run, speed=1000):
        if run:
            self.belt_motor.run(speed)
        else:
            self.belt_motor.stop()
 
 
# Example usage
robot = Robot()
robot.collect(1)
robot.wait_until_pressed()
robot.turn(90)               # Turn 90 degrees clockwise
robot.go_straight(720)       # Go straight for 720 degrees of wheel rotation
# robot.drive_until_obstacle() # Move forward until an obstacle is detected within 200 mm
robot.turn_to_angle(180)     # Turn to face 180 degrees
 
