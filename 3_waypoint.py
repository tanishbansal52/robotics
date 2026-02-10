import time
import brickpi3
import math
import random

BP = brickpi3.BrickPi3()

# CHANGE THESE
DEBUG = False
forward_sleep = 2
turn_sleep = 2
metre_degrees = 1750

def reset_motor():
    # Reseting motor
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))

def set_power_limit(power_limit):
    # Setting power limits
    BP.set_motor_limits(BP.PORT_B, power_limit)
    BP.set_motor_limits(BP.PORT_C, power_limit)

def go_forward(dist):
    BP.set_motor_limits(BP.PORT_B, 71)
    BP.set_motor_limits(BP.PORT_C, 70)
    reset_motor()
    forward_degrees = dist * metre_degrees
    
    BP.set_motor_position(BP.PORT_B | BP.PORT_C, forward_degrees)
    if DEBUG:
        print("=======FORWARD INFO========")
        print_motor_info()
    time.sleep(forward_sleep)

def turn(angle):
    turn_degrees = angle * (270 / (math.pi / 2.0))
    # turn
    BP.set_motor_limits(BP.PORT_B, 50)
    BP.set_motor_limits(BP.PORT_C, 50)
    reset_motor()
    BP.set_motor_position(BP.PORT_C, turn_degrees)
    BP.set_motor_position(BP.PORT_B, -turn_degrees)
    if DEBUG:
        print("=======TURN INFO========")
        print_motor_info()
    time.sleep(turn_sleep)
    
def print_motor_info():
    print("---------B--------")
    print(BP.get_motor_status(BP.PORT_B))
    print(BP.get_motor_encoder(BP.PORT_B))
    print("---------C--------")
    print(BP.get_motor_status(BP.PORT_C))
    print(BP.get_motor_encoder(BP.PORT_C))

def estimate_pose_from_particles():
    """
    Compute mean pose (x_bar, y_bar, theta_bar) from global particle set.
    Particles are (x_i, y_i, theta_i, w_i) in world metres / radians.
    """
    sum_x = sum(w * x for (x, y, th, w) in particles)
    sum_y = sum(w * y for (x, y, th, w) in particles)
    # For angle, average on unit circle to handle wrap-around
    sum_cos = sum(w * math.cos(th) for (x, y, th, w) in particles)
    sum_sin = sum(w * math.sin(th) for (x, y, th, w) in particles)
    theta_bar = math.atan2(sum_sin, sum_cos)
    return sum_x, sum_y, theta_bar

def navigateToWaypoint(Wx, Wy):
    x, y, theta = estimate_pose_from_particles()

    dx = Wx - x
    dy = Wy - y

    alpha = math.atan2(dy, dx) 

    beta = alpha - theta
    while beta > math.pi:
        beta -= 2.0 * math.pi
    while beta <= -math.pi:
        beta += 2.0 * math.pi

    d = math.hypot(dx, dy)

    turn_on_spot(beta)
    drive_straight(d)
    update_part_turn(beta)
    update_part_forward(d)

    x_new, y_new, theta_new = estimate_pose_from_particles()
    return x_new, y_new, theta_new

    

def main():
    try:
        x0, y0, theta = 0, 0, 0
        go_forward()
        turn()
        time.sleep(0.5)

    except KeyboardInterrupt:
        BP.reset_all()
    
    BP.reset_all()

main()
   
