from __future__ import print_function 
from __future__ import division 

import time
import brickpi3
import math
import random                              

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.NXT_ULTRASONIC)

# CHANGE THESE
DEBUG = False
forward_sleep = 2
turn_sleep = 2
forward_degrees = 900 / 4
x_shift = 10
y_shift = 45
num_particles = 100
scale = 15
w_i = 1/num_particles
particles = [(0.0, 0.0, 0.0, w_i)] * num_particles

######################################################################

# Functions to generate some dummy particles data:
def calcX(t):
    return random.gauss(80,3) + 70*(math.sin(t)) # in cm

def calcY(t):
    return random.gauss(70,3) + 60*(math.sin(2*t)) # in cm

def calcW():
    return random.random()

def calcTheta():
    return random.randint(0,360)

# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size    # in cm
        self.canvas_size = 768         # in pixels
        self.margin      = 0.05*map_size
        self.scale       = self.canvas_size/(map_size+2*self.margin)

    def drawLine(self,line):
        x1 = self.__screenX(line[0])
        y1 = self.__screenY(line[1])
        x2 = self.__screenX(line[2])
        y2 = self.__screenY(line[3])
        print ("drawLine:" + str((x1,y1,x2,y2)))

    def drawParticles(self,data):
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data]
        print ("drawParticles:" + str(display))

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = []

    def add_wall(self,wall):
        self.walls.append(wall)

    def clear(self):
        self.walls = []

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall)

# Simple Particles set
class Particles:
    def __init__(self):
        self.n = 10    
        self.data = []

    def update(self, t):
        self.data = [(calcX(t), calcY(t), calcTheta(), calcW()) for i in range(self.n)]
    
    def draw(self):
        canvas.drawParticles(self.data)
        
global canvas 
canvas = Canvas()

global mymap 
mymap = Map()
        
def draw_canvas():
    # Definitions of walls
    # a: O to A
    # b: A to B
    # c: C to D
    # d: D to E
    # e: E to F
    # f: F to G
    # g: G to H
    # h: H to O
    mymap.add_wall((0,0,0,168))        # a
    mymap.add_wall((0,168,84,168))     # b
    mymap.add_wall((84,126,84,210))    # c
    mymap.add_wall((84,210,168,210))   # d
    mymap.add_wall((168,210,168,84))   # e
    mymap.add_wall((168,84,210,84))    # f
    mymap.add_wall((210,84,210,0))     # g
    mymap.add_wall((210,0,0,0))        # h
    mymap.draw()

    particles = Particles()

    t = 0
    while True:
        particles.update(t)
        particles.draw()
        t += 0.05
        time.sleep(0.05)
        
######################################################################

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
    if DEBUG:
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
    
def update_part_forward(D = 10):
    sigma_e = 0.1
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
    sigma_g = 0.02

    for i, p in enumerate(particles):
        x, y, th, w = p
        g = random.gauss(0.0, sigma_g)
        th_new = th + alpha + g
        particles[i] = ((x, y, th_new, w))
    
def draw_line(x0, y0, x1, y1):
    print("drawLine:" + 
            str(
                ((x_shift+x0)*scale, 
                 (y_shift-y0)*scale, 
                 (x_shift+x1)*scale, 
                 (y_shift-y1)*scale
                )
               )
         )

def draw_particles():
    new_p = [((x_i + 10) * scale,  (y_shift - y_i) * scale, theta_i, w) for (x_i,  y_i, theta_i, w) in particles]
    print("drawParticles:" + str(new_p))
    
def get_sonar_reading():
    try:
        value = BP.get_sensor(BP.PORT_4)
        return value                        
    except brickpi3.SensorError as error:
        print(error)

    time.sleep(0.02)
    
def find_wall(x, y, theta):
    if y < 0 or y > 215 or x < 0 or x > 215:
        print("X or Y failed")
        return
    m = 1 / math.arctan(theta)
        

def main():
    try:
        x0, y0, theta = 0, 0, 0
        x1, y1 = x0, y0
        toAdd = [(10, 0), (0, 10), (-10, 0), (0, -10)]
        for i in range(4):
            for j in range(4):
                go_forward()
                x1 += toAdd[i][0]
                y1 += toAdd[i][1]
                draw_line(x0, y0, x1, y1)
                print(x1, y1)
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

#main()

draw_canvas()



   
