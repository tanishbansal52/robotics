from __future__ import print_function 
from __future__ import division 

import time
import brickpi3
import math
import random                              

BP = brickpi3.BrickPi3()

BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.NXT_ULTRASONIC)
time.sleep(0.2)

# Wavpoint navigation stuff
POSITIONS = [(84, 30), (180, 30), (180, 54), (138, 54), (138, 168), (114, 168), (114, 84), (84, 84), (84, 30)]
position = (POSITIONS[0][0], POSITIONS[0][1], 0)

POSITION_TOLERANCE = 3.0
ANGLE_TOLERANCE = 2 * math.pi / 180

# Settings
DEBUG = False
forward_sleep = 5
turn_sleep = 2
metre_degrees = 2187
SENSOR_OFFSET_FROM_CENTRE = 7 # 7cm off from centre of wheels
######################################################################

# Functions to generate some dummy particles data:

sigma_e = 2.5
sigma_f = 0.02
sigma_g = 0.01
    
    
# D is in cm
def calc_particle_forward(x, y, theta, D):
    e = random.gauss(0.0, sigma_e)
    f = random.gauss(0.0, sigma_f)
    D_noisy = D + e
    x_new = x + D_noisy * math.cos(theta)
    y_new = y + D_noisy * math.sin(theta)
    theta_new = theta + f
    return (x_new, y_new, theta_new)

def calc_particle_on_turn(x, y, theta, alpha):
    g = random.gauss(0.0, sigma_g)
    theta_new = theta + alpha + g
    return (x, y, theta_new)
    
def calcW():
    return random.random()

def calcTheta():
    return random.randint(0,360) * math.pi / 180.0

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
        self.n = 100
        self.data = []
        for i in range(self.n):
            x = position[0]
            y = position[1]
            theta = position[2]
            w = 1/self.n
            self.data.append((x, y, theta, w))

    def update(self, d, d_theta):
        motion_particles = []
        total_weight = 0
        z = 0
        for i in range(5):
            z += get_sonar_reading()
        z /= 5
        
        for x, y, theta, w in self.data:
            if d_theta == 0:
                x, y, theta = calc_particle_forward(x, y, theta, d)
            else:
                x, y, theta = calc_particle_on_turn(x, y, theta, d_theta)
            w = calculate_likelihood(x, y, theta, z)
            total_weight += w
            motion_particles.append((x, y, theta, w))
            
        #print(motion_particles)

        # Normalising
        normalised_particles = []
        for x, y, theta, w in motion_particles:
            normalised_particles.append((x, y, theta, w/total_weight))
            
        # Resampling
        self.data = []
        cum_w = [0.0] * self.n
        cum_w[0] = normalised_particles[0][3]
        for i in range(1, self.n):
            cum_w[i] = cum_w[i-1] + normalised_particles[i][3]
    
        for i in range(self.n):
            r = random.uniform(0, 1)
            # Find j where cum_w[j-1] < r <= cum_w[j]
            for j in range(self.n):
                if cum_w[j] >= r:
                    x, y, theta, _ = normalised_particles[j]
                    self.data.append((x, y, theta, 1/self.n))
                    break
        
    def get_estimate_pos(self):
        # Calculate the weighted mean of the particles
        mean_x = 0
        mean_y = 0
        mean_sin = 0
        mean_cos = 0
        total_weight = 0
        
        for x, y, theta, w in self.data:
            mean_x += x * w
            mean_y += y * w
            # Use vector addition for angles to handle the 0/360 wrap-around correctly
            mean_sin += math.sin(theta) * w
            mean_cos += math.cos(theta) * w
            total_weight += w
            
        if total_weight == 0:
            return (0,0,0) # Should not happen if normalized
            
        mean_x /= total_weight
        mean_y /= total_weight
        mean_theta = math.atan2(mean_sin, mean_cos)
        
        return (mean_x, mean_y, mean_theta)

        
    def draw(self):
        canvas.drawParticles(self.data)
        
global canvas 
canvas = Canvas()

global mymap 
mymap = Map()

global particles
particles = Particles()

        
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

def draw_canvas_particles(dist, d_theta):
    particles.update(dist, d_theta)
    particles.draw()
        
######################################################################

def get_sonar_reading():
    try:
        value = BP.get_sensor(BP.PORT_4)
        time.sleep(0.02)
        return value + SENSOR_OFFSET_FROM_CENTRE
    except brickpi3.SensorError as error:
        print("Sonar error:", error)
        return None
    
def find_wall(x, y, theta):
    if y < 0 or y > 215 or x < 0 or x > 215:
        print("X or Y failed")
        return
    walls = {"a":(0,0,0,168), "b":(0,168,84,168), "c":(84,126,84,210), "d":(84,210,168,210),
             "e":(168,210,168,84), "f":(168,84,210,84), "g":(210,84,210,0), "h":(210,0,0,0)}
    dists = {}
    min_k = ""
    min_m = float('inf')
    for k, w in walls.items():
        ax, ay, bx, by = w
        m = 0 
        if ((by - ay) * math.cos(theta) - (bx - ax) * math.sin(theta)) != 0:
            m = ((by - ay) * (ax - x) - (bx - ax) * (ay - y)) / ((by - ay) * math.cos(theta) - (bx - ax) * math.sin(theta))
        target_x = x + m * math.cos(theta)
        target_y = y + m * math.sin(theta)
        if ax > bx:
            ax, bx = bx, ax
        if ay > by:
            ay, by = by, ay
        if m >= 0 and (ax <= target_x <= bx) and (ay <= target_y <= by):
            dists[k] = m
            if m < min_m:
                min_k = k
                min_m = m
    return (min_k, min_m)
    
def calculate_likelihood(x, y, theta, z):
    wall, m = find_wall(x, y, theta)
    st_dev = 2.5
    K = 0.0001
    likelihood = math.exp( (-((z-m)**2)) / (2*(st_dev)**2) ) + K
    return likelihood
    
##############################################################################
##############################################################################

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
    time.sleep(turn_sleep * (abs(angle) / (math.pi / 2.0)))
    
def print_motor_info():
    print("---------B--------")
    print(BP.get_motor_status(BP.PORT_B))
    print(BP.get_motor_encoder(BP.PORT_B))
    print("---------C--------")
    print(BP.get_motor_status(BP.PORT_C))
    print(BP.get_motor_encoder(BP.PORT_C))

    
def wrap_to_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a
    
def navigateToWaypoint(Wx, Wy):
    print("in nav")
    global position
    x, y, theta = position
    est_theta = theta
    
    while True:
        dx = Wx - x
        dy = Wy - y
        to_move = math.hypot(dx, dy)

        if to_move <= POSITION_TOLERANCE:
            break
            
        desired = math.atan2(dy, dx)
        heading_err = wrap_to_pi(desired - est_theta)

        # TURN
        if abs(heading_err) > ANGLE_TOLERANCE:     
            print("TURNING in NavWaypoint")
            turn(heading_err)
            draw_canvas_particles(0, heading_err)
            time.sleep(1)
        else: # FORWARD
            print("MOVING in NavWaypoint")
            move_by = min(to_move, 20)
            if move_by <= POSITION_TOLERANCE:
                break
            go_forward(move_by/100)
            draw_canvas_particles(move_by, 0)
            
        est_x, est_y, est_theta = particles.get_estimate_pos()
        position = (est_x, est_y, est_theta)
        time.sleep(2)
            
        x = est_x
        y = est_y
        theta = est_theta
        
    return
    
##############################################################################
##############################################################################

def main():
    print("in main")
    try:
        draw_canvas()
        reset_motor()
        
        for i, (wx, wy) in enumerate(POSITIONS[1:]):
            print(f"Navigating to waypoint {i+1}: ({wx}, {wy})")
            navigateToWaypoint(wx, wy)
            time.sleep(2)

    except KeyboardInterrupt:
        BP.reset_all()
    
    BP.reset_all()

main()
   
