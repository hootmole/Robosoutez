from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent
from ev3dev2.sensor.lego import GyroSensor
from time import sleep

def turn_robot_fast(turn_angle):
    # Initialize the gyro sensor and motors
    gyro = GyroSensor()
    left_motor = LargeMotor(OUTPUT_A)
    right_motor = LargeMotor(OUTPUT_B)
    
    # Reset the gyro sensor angle to 0
    gyro.reset()
    sleep(0.2)  # Minimal wait for gyro to stabilize
    
    # Determine initial high speed and the direction
    initial_speed = 50  # Start with high speed for quick turning
    final_speed = 10    # Lower speed for fine adjustment near target
    direction = 1 if turn_angle > 0 else -1  # Clockwise or counterclockwise

    # Start turning at initial speed
    left_motor.run_forever(speed_sp=SpeedPercent(initial_speed * direction))
    right_motor.run_forever(speed_sp=SpeedPercent(-initial_speed * direction))
    
    # Gradually slow down as it approaches the target
    while abs(gyro.angle) < abs(turn_angle) - 10:  # Slow down 10 degrees before target
        pass
    
    # Reduce speed for fine adjustment close to target
    left_motor.run_forever(speed_sp=SpeedPercent(final_speed * direction))
    right_motor.run_forever(speed_sp=SpeedPercent(-final_speed * direction))
    
    # Continue turning until target angle reached with small tolerance
    while abs(gyro.angle) < abs(turn_angle):
        pass
    
    # Stop motors once the target angle is reached
    left_motor.stop()
    right_motor.stop()
    
    print(f"Turn completed to approximately {gyro.angle} degrees.")

# Example usage:
# turn_robot_fast(90)  # Quickly turns the robot by 90 degrees
