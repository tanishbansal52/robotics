import time
import brickpi3
import math
import random

BP = brickpi3.BrickPi3()

# Positions
TARGET = (0.1, 0.1)
position = (0, 0, 0)

# Settings
DEBUG = False
forward_sleep = 5
turn_sleep = 2
metre_degrees = 2187
#2187.5


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
    print(f"Travelling {dist} metres")
    forward_degrees = dist * metre_degrees
    
    BP.set_motor_position(BP.PORT_B | BP.PORT_C, forward_degrees)
    if DEBUG:
        print("=======FORWARD INFO========")
        print_motor_info()
    time.sleep(forward_sleep * dist)

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
    time.sleep(turn_sleep * (angle / (math.pi / 2.0)))
    
def print_motor_info():
    print("---------B--------")
    print(BP.get_motor_status(BP.PORT_B))
    print(BP.get_motor_encoder(BP.PORT_B))
    print("---------C--------")
    print(BP.get_motor_status(BP.PORT_C))
    print(BP.get_motor_encoder(BP.PORT_C))

def navigateToWaypoint(Wx, Wy):
    print("in nav")
    global position
    x, y, theta = position

    dx = Wx - x
    dy = Wy - y

    alpha = math.atan2(dy, dx)
    d_theta = alpha - theta
    
    
    print(f"Navigating from {position} by ({dx}, {dy}, {d_theta})")
          
    while d_theta > math.pi:
        d_theta -= 2.0 * math.pi
    while d_theta <= -math.pi:
        d_theta += 2.0 * math.pi

    d = math.hypot(dx, dy)

    turn(d_theta)
    go_forward(d)

    position = (Wx, Wy, alpha)
    return
    
def main():
    print("in main")
    try:
        reset_motor()
        wx, wy = TARGET
        navigateToWaypoint(wx, wy)

    except KeyboardInterrupt:
        BP.reset_all()
    
    BP.reset_all()

main()
   
