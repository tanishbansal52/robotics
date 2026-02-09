import time
import brickpi3

BP = brickpi3.BrickPi3()

# CHANGE THESE
forward_sleep = 2
turn_sleep = 2
forward_degrees = 875


def reset_motor():
    # Reseting motor
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))

def set_power_limit(power_limit):
    # Setting power limits
    BP.set_motor_limits(BP.PORT_B, power_limit)
    BP.set_motor_limits(BP.PORT_C, power_limit)

def go_forward():
    # forward
    BP.set_motor_limits(BP.PORT_B, 71)
    BP.set_motor_limits(BP.PORT_C, 70)
    reset_motor()
    BP.set_motor_position(BP.PORT_B | BP.PORT_C, forward_degrees)
    print("=======FORWARD INFO========")
    print_motor_info()
    time.sleep(forward_sleep)

def turn():
    turn_degrees = 260
    # turn
    BP.set_motor_limits(BP.PORT_B, 50)
    BP.set_motor_limits(BP.PORT_C, 50)
    reset_motor()
    BP.set_motor_position(BP.PORT_B, turn_degrees)
    BP.set_motor_position(BP.PORT_C, -turn_degrees)
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

    
def main():
    try:
        for i in range(4):
            go_forward()
            turn()

    except KeyboardInterrupt:
        BP.reset_all()
    
    BP.reset_all()

main()
# BP.reset_all()
   
