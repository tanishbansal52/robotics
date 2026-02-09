import time
import brickpi3

BP = brickpi3.BrickPi3()

    
# CHANGE THESE
short_time = 0.02
sleep_time = 2
move_degrees = 875

def reset_motor():
    # Reseting motor
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))
    time.sleep(0.1)

def set_power_limit(power_limit):
    # Setting power limits
    BP.set_motor_limits(BP.PORT_B, power_limit)
    BP.set_motor_limits(BP.PORT_C, power_limit)
    
    
def set_motor_power(power_limit):
    # Setting power limits
    BP.set_motor_power(BP.PORT_B, power_limit)
    BP.set_motor_power(BP.PORT_C, power_limit)

def move(dir):
    
    #set_motor_power(0)
    reset_motor()
    
    # forward
    BP.set_motor_limits(BP.PORT_B | BP.PORT_C, 70)
    
    if dir == 1:
        BP.set_motor_power(BP.PORT_B | BP.PORT_C, dir*55)
    else:
        print("AHOY BACK")
        BP.set_motor_position(BP.PORT_B | BP.PORT_C, -move_degrees)
        time.sleep(2)
        
        print(BP.get_motor_status(BP.PORT_B))
        print(BP.get_motor_status(BP.PORT_C))
        while True:
            stat_b = BP.get_motor_status(BP.PORT_B)
            stat_c = BP.get_motor_status(BP.PORT_C)
            diff_b = abs(stat_b[2] + move_degrees)
            diff_c = abs(stat_c[2] + move_degrees)
            if diff_b < 10 or diff_c < 10:
                break
            time.sleep(sleep_time)
            continue
        print("AHOY EXIT")

def turn(dir):
    print("AYYY TURN UR BOOTY")
    turn_degrees = 130
    
    # turn
    BP.set_motor_limits(BP.PORT_B, 50)
    BP.set_motor_limits(BP.PORT_C, 50)
    
    #set_motor_power(0)
    reset_motor()
    
    BP.set_motor_position(BP.PORT_B, dir * turn_degrees)
    BP.set_motor_position(BP.PORT_C, dir * -turn_degrees)
    time.sleep(1)
    
    while not BP.get_motor_status(BP.PORT_B)[3] == 0:
        print(BP.get_motor_status(BP.PORT_B))
        time.sleep(short_time)

    print("AHOY TURN EXIT")
    time.sleep(1)
    
def print_motor_info():
    print("---------B--------")
    print(BP.get_motor_status(BP.PORT_B))
    print(BP.get_motor_encoder(BP.PORT_B))
    print("---------C--------")
    print(BP.get_motor_status(BP.PORT_C))
    print(BP.get_motor_encoder(BP.PORT_C))
    
def check_bumper():
    right = BP.get_sensor(BP.PORT_3)
    left = BP.get_sensor(BP.PORT_4)
    if left:
        print("AYYY CRASH LEFT")
        set_motor_power(0)
        move(-1)
        time.sleep(1)
        turn(1)
    if not left and right:
        print("AYYY CRASH RIGHT")
        set_motor_power(0)
        move(-1)
        time.sleep(1)
        turn(-1)
    time.sleep(short_time)

       
    
def main():
    BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.TOUCH)
    time.sleep(2)
    try:
        while True:
            move(1)
            check_bumper()
    except KeyboardInterrupt:
        BP.reset_all()
    
    BP.reset_all()

main()
# BP.reset_all()
