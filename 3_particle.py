import time
import brickpi3

BP = brickpi3.BrickPi3()

# CHANGE THESE
forward_sleep = 2
turn_sleep = 2
forward_degrees = 875 / 4


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
    # orignal 260
    turn_degrees = 270
    # turn
    BP.set_motor_limits(BP.PORT_B, 50)
    BP.set_motor_limits(BP.PORT_C, 50)
    reset_motor()
    BP.set_motor_position(BP.PORT_C, turn_degrees)
    BP.set_motor_position(BP.PORT_B, -turn_degrees)
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
    
def draw_line(x0, y0, x1, y1):
    print("drawLine:" + str((x0*10, 400-y0*10, x1*10, 400-y1*10)))

def draw_particles(x, y, theta):
    print("drawParticles:" + str((x*10, y*10, theta)))

def main():
    try:
        x0, y0 = 0, 0
        x1, y1 = 0, 0 
        theta = 0
        toAdd = [(10, 0), (0, 10), (-10, 0), (0, -10)]
        for i in range(4):
            for j in range(4):
                go_forward()
                x1 += toAdd[i][0]
                y1 += toAdd[i][1]
                draw_line(x0, y0, x1, y1)
                x0 = x1
                y0 = y1
                time.sleep(0.5)
                draw_particles(x1, y1, theta)
            turn()
            theta += 90

    except KeyboardInterrupt:
        BP.reset_all()
    
    BP.reset_all()

main()
# BP.reset_all()
   



