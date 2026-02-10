import time
import brickpi3
import math
import random

BP = brickpi3.BrickPi3()

# CHANGE THESE
forward_sleep = 2
turn_sleep = 2
forward_degrees = 875 / 4
y_max = 45
num_particles = 100
scale = 15
w_i = 1/num_particles
particles = [(0.0, 0.0, 0.0, w_i)] * num_particles


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
    
def update_part_forward(D = 10):
    sigma_e = 0.01
    sigma_f = 0.02
    
    for i, p in enumerate(particles):
        x, y, th, w = p
        e = random.gauss(0.0, sigma_e)
        f = random.gauss(0.0, sigma_f)
        D_noisy = D + e
        x_new = x + D_noisy * math.cos(th)
        y_new = y + D_noisy * math.sin(th)
        th_new = th + f
        particles[i] = ((x_new, y_new, th_new, w))
    
def update_part_turn(alpha = math.pi/2):
    sigma_g = 0.002

    for i, p in enumerate(particles):
        x, y, th, w = p
        g = random.gauss(0.0, sigma_g)
        th_new = th + alpha + g
        particles[i] = ((x, y, th_new, w))
    
def draw_line(x0, y0, x1, y1):
    print("drawLine:" + 
            str(
                (x0*scale, 
                 (y_max-y0)*scale, 
                 x1*scale, 
                 (y_max-y1)*scale
                )
            )
         )

def draw_particles():
    new_p = [(scale*(x_i+10),  (y_max-y_i)*scale, theta_i, w) for (x_i,  y_i, theta_i, w) in particles]
    print("drawParticles:" + str(new_p))
    
def main():
    try:
        x0, y0 = 10, 0
        x1, y1 = x0, y0
        toAdd = [(10, 0), (0, 10), (-10, 0), (0, -10)]
        for i in range(4):
            for j in range(4):
                go_forward()
                x1 += toAdd[i][0]
                y1 += toAdd[i][1]
                draw_line(x0, y0, x1, y1)
                update_part_forward()
                draw_particles()
                x0 = x1
                y0 = y1
                time.sleep(0.5)
            turn()
            update_part_turn()
            draw_particles()
            time.sleep(0.5)

    except KeyboardInterrupt:
        BP.reset_all()
    
    BP.reset_all()

main()
# BP.reset_all()
   



