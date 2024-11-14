
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor.lego import UltrasonicSensor
from time import sleep

class Robot:
    def __init__(self, left_motor_port=OUTPUT_A, right_motor_port=OUTPUT_B, gyro_port='in1'):
        # Initialize motors and sensors
        self.left_motor = LargeMotor(left_motor_port)
        self.right_motor = LargeMotor(right_motor_port)
        self.gyro = GyroSensor(gyro_port)
        
        # Reset gyro angle to 0 on initialization
        self.gyro.reset()
        sleep(0.2)  # Minimal wait for gyro stabilization
        
    def turn(self, angle, speed=50, fine_speed=10):
        """
        Turns the robot by a specified angle using the gyro sensor.
        :param angle: Angle in degrees (positive for clockwise, negative for counterclockwise)
        :param speed: Initial speed for fast turning
        :param fine_speed: Final speed for fine adjustment
        """
        direction = 1 if angle > 0 else -1
        self.gyro.reset()
        
        # Start turning at high speed
        self.left_motor.run_forever(speed_sp=SpeedPercent(speed * direction))
        self.right_motor.run_forever(speed_sp=SpeedPercent(-speed * direction))
        
        # Gradually slow down as the target angle is approached
        while abs(self.gyro.angle) < abs(angle) - 10:
            pass
        
        # Reduce speed for fine adjustment near the target
        self.left_motor.run_forever(speed_sp=SpeedPercent(fine_speed * direction))
        self.right_motor.run_forever(speed_sp=SpeedPercent(-fine_speed * direction))
        
        # Stop when target angle is reached within a small tolerance
        while abs(self.gyro.angle) < abs(angle):
            pass
        
        self.stop()
    
    def go_straight(self, distance, speed=30):
        """
        Moves the robot straight for a specified distance.
        :param distance: Distance in degrees of wheel rotation
        :param speed: Speed of motors
        """
        # Reset motor positions
        self.left_motor.position = 0
        self.right_motor.position = 0
        
        # Run both motors forward
        self.left_motor.run_to_rel_pos(position_sp=distance, speed_sp=SpeedPercent(speed))
        self.right_motor.run_to_rel_pos(position_sp=distance, speed_sp=SpeedPercent(speed))
        
        # Wait until both motors complete the motion
        self.left_motor.wait_while('running')
        self.right_motor.wait_while('running')
    
    def stop(self):
        """
        Stops both motors immediately.
        """
        self.left_motor.stop()
        self.right_motor.stop()

    def drive_until_obstacle(self, speed=30, distance=20):
        """
        Moves forward until an obstacle is detected within a certain distance.
        :param speed: Speed of the motors
        :param distance: Distance to obstacle in cm
        """
        us_sensor = UltrasonicSensor()  # Make sure to have the ultrasonic sensor connected
        us_sensor.mode = 'US-DIST-CM'
        
        # Move forward until an obstacle is detected within the specified distance
        self.left_motor.run_forever(speed_sp=SpeedPercent(speed))
        self.right_motor.run_forever(speed_sp=SpeedPercent(speed))
        
        while us_sensor.distance_centimeters > distance:
            pass
        
        self.stop()

    def turn_to_angle(self, target_angle, speed=10):
        """
        Turns the robot to face a specific angle (relative to the initial reset angle).
        :param target_angle: Absolute target angle relative to the initial reset
        :param speed: Turning speed
        """
        current_angle = self.gyro.angle
        angle_to_turn = target_angle - current_angle
        self.turn(angle_to_turn, speed=speed)

# Example usage
robot = Robot()
robot.turn(90)               # Turn 90 degrees clockwise
robot.go_straight(720)       # Go straight for 720 degrees of wheel rotation
robot.drive_until_obstacle() # Move forward until an obstacle is detected within 20 cm
robot.turn_to_angle(180)     # Turn to face 180 degrees
