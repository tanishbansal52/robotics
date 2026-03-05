#!/usr/bin/env python3
# ============================================================
# DWA Navigation Challenge - BrickPi3
# Combines: 6_planning.py (DWA core) + 5_camerahomography.py
#           + 6_canSegmentation.py (sensing)
# ============================================================

import brickpi3
import time
import math
import numpy as np
import cv2
from picamera2 import Picamera2

# ─── BrickPi & Motor Ports ───────────────────────────────────────────────────
BP = brickpi3.BrickPi3()
MOTOR_LEFT  = BP.PORT_B
MOTOR_RIGHT = BP.PORT_C

# ─── Physical Robot Parameters (MEASURE YOUR ROBOT) ─────────────────────────
WHEEL_RADIUS    = 0.028   # metres – standard LEGO wheel ~28mm
WHEEL_SEP       = 0.155   # metres W – distance between wheel centres
W               = WHEEL_SEP

def mps_to_dps(v):
    """Convert wheel speed m/s -> degrees per second for BP.set_motor_dps."""
    return v / WHEEL_RADIUS * (180.0 / math.pi)

# ─── DWA Tuning Parameters ───────────────────────────────────────────────────
MAXVELOCITY     = 0.40   # m/s  – increase once reliable                       # 0.40
MAXACCELERATION = 0.50   # m/s² – how fast velocity can change per step
BARRIER_RADIUS  = 0.05   # m    – cola can ~3cm radius, 6cm for safety marging # 0.05
DT              = 0.10   # s    – main control loop timestep

SAFEDIST        = 0.15   # tighter — allows passing 40cm gaps confidently
OBSTACLE_WEIGHT = 30     # higher — stronger avoidance signal
FORWARD_WEIGHT  = 10
TAU             = 1.0    # shorter lookahead — more reactive, less phantom-avoidance

ROBOT_RADIUS    = 0.15   # m    – robot footprint radius


# Target: well beyond finish line so robot keeps driving through
TARGET_X, TARGET_Y = 4.5, 0.0

# ─── Camera Setup ────────────────────────────────────────────────────────────
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(1.0)  # let camera warm up

# ─── Ground Plane Homography ─────────────────────────────────────────────────
# REPLACE THESE WITH YOUR OWN CALIBRATED CORRESPONDENCES from Practical 5
# Format: (x_cm_forward, y_cm_lateral, u_pixel, v_pixel)
# x is forward from robot, y is left-positive lateral
(x1, y1, u1, v1) = (28, 18, 29, 332)
(x2, y2, u2, v2) = (28, -16, 586, 316)
(x3, y3, u3, v3) = (68, -26, 530, 143)
(x4, y4, u4, v4) = (68, 26, 115, 153)

A = np.array([
    [x1,y1,1, 0, 0,0, -u1*x1,-u1*y1],
    [ 0, 0,0,x1,y1,1, -v1*x1,-v1*y1],
    [x2,y2,1, 0, 0,0, -u2*x2,-u2*y2],
    [ 0, 0,0,x2,y2,1, -v2*x2,-v2*y2],
    [x3,y3,1, 0, 0,0, -u3*x3,-u3*y3],
    [ 0, 0,0,x3,y3,1, -v3*x3,-v3*y3],
    [x4,y4,1, 0, 0,0, -u4*x4,-u4*y4],
    [ 0, 0,0,x4,y4,1, -v4*x4,-v4*y4],
])
b_h = np.array([u1,v1,u2,v2,u3,v3,u4,v4])
R_h, _, _, _ = np.linalg.lstsq(A, b_h, rcond=None)
H    = np.array([[R_h[0],R_h[1],R_h[2]],
                 [R_h[3],R_h[4],R_h[5]],
                 [R_h[6],R_h[7],1.0]])
HInv = np.linalg.inv(H)

def pixel_to_robot_m(u, v):
    """Pixel (u,v) -> robot-frame (x_m forward, y_m lateral) in metres."""
    uvec = np.array([u, v, 1.0])
    xvec = HInv.dot(uvec)
    x_cm = xvec[0] / xvec[2]
    y_cm = xvec[1] / xvec[2]
    return x_cm / 100.0, y_cm / 100.0

# ─── Robot State ─────────────────────────────────────────────────────────────
robot_x     = 0.0   # world frame metres, forward
robot_y     = 0.0   # world frame metres, lateral
robot_theta = 0.0   # world frame heading (0 = straight ahead)
vL = 0.0            # current left wheel speed m/s
vR = 0.0            # current right wheel speed m/s

# World-frame obstacle list: [[wx, wy], ...]
obstacles = []
MERGE_DIST = 0.20   # merge observations within 20cm into same obstacle

# Encoder state for odometry
BP.reset_motor_encoder(MOTOR_LEFT)
BP.reset_motor_encoder(MOTOR_RIGHT)
enc_L_prev = 0
enc_R_prev = 0

# ─── Odometry ────────────────────────────────────────────────────────────────
def update_odometry():
    global robot_x, robot_y, robot_theta, enc_L_prev, enc_R_prev
    try:
        enc_L = BP.get_motor_encoder(MOTOR_LEFT)
        enc_R = BP.get_motor_encoder(MOTOR_RIGHT)
    except Exception:
        return

    d_L = (enc_L - enc_L_prev) * (math.pi / 180.0) * WHEEL_RADIUS
    d_R = (enc_R - enc_R_prev) * (math.pi / 180.0) * WHEEL_RADIUS
    enc_L_prev = enc_L
    enc_R_prev = enc_R

    d      = (d_L + d_R) / 2.0
    dtheta = (d_R - d_L) / W
    # Midpoint integration for better accuracy
    robot_x     += d * math.cos(robot_theta + dtheta / 2.0)
    robot_y     += d * math.sin(robot_theta + dtheta / 2.0)
    robot_theta += dtheta

# ─── Camera Sensing ──────────────────────────────────────────────────────────
def sense_obstacles():
    global obstacles
    obstacles = []  # ← CLEAR every frame, fresh detection only
    try:
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = (cv2.inRange(hsv, np.array([0,  50, 50]), np.array([10, 255,255])) +
                cv2.inRange(hsv, np.array([170,50, 50]), np.array([180,255,255])))

        new_robot_pts = []
        for u in range(0, 640, 10):
            red_rows = np.where(mask[:, u] > 0)[0]
            if len(red_rows) == 0:
                continue
            v_bottom = int(np.max(red_rows))
            rx, ry = pixel_to_robot_m(u, v_bottom)
            if 0.15 < rx < 2.0 and abs(ry) < 1.5:
                new_robot_pts.append((rx, ry))

        # Merge nearby points in robot frame first (cleaner than world frame)
        merged_pts = []
        for (rx, ry) in new_robot_pts:
            found = False
            for mp in merged_pts:
                if math.hypot(mp[0]-rx, mp[1]-ry) < 0.12:
                    mp[0] = (mp[0]+rx)/2; mp[1] = (mp[1]+ry)/2
                    found = True; break
            if not found:
                merged_pts.append([rx, ry])

        # Convert merged robot-frame points to world frame
        for (rx, ry) in merged_pts:
            wx = robot_x + rx*math.cos(robot_theta) - ry*math.sin(robot_theta)
            wy = robot_y + rx*math.sin(robot_theta) + ry*math.cos(robot_theta)
            obstacles.append([wx, wy])

    except Exception as e:
        print("Camera error:", e)


# ─── DWA Core (from 6_planning.py, Pygame stripped) ──────────────────────────
def predict_position(vL, vR, x, y, theta, deltat):
    """Predict robot pose after deltat seconds at constant (vL, vR)."""
    if abs(vL - vR) < 1e-6:          # straight line
        return (x + vL * deltat * math.cos(theta),
                y + vL * deltat * math.sin(theta),
                theta)
    elif abs(vL + vR) < 1e-6:        # pure rotation
        return (x, y, theta + (vR - vL) * deltat / W)
    else:                             # general arc
        R_arc  = W / 2.0 * (vR + vL) / (vR - vL)
        dtheta = (vR - vL) * deltat / W
        xn = x + R_arc * (math.sin(dtheta + theta) - math.sin(theta))
        yn = y - R_arc * (math.cos(dtheta + theta) - math.cos(theta))
        return xn, yn, theta + dtheta

def closest_obstacle_dist(x, y):
    """Distance from (x,y) to nearest obstacle surface (negative = inside)."""
    if not obstacles:
        return 1000.0
    return min(math.hypot(obs[0]-x, obs[1]-y) - BARRIER_RADIUS - ROBOT_RADIUS
               for obs in obstacles)

def dwa_step():
    global vL, vR
    best_benefit = -1e9
    vL_best, vR_best = vL, vR

    # 5 candidates per wheel instead of 3
    dv = MAXACCELERATION * DT
    vL_options = [vL + i*dv/2 for i in range(-2, 3)]
    vR_options = [vR + i*dv/2 for i in range(-2, 3)]

    for vLp in vL_options:
        for vRp in vR_options:
            if not (-MAXVELOCITY <= vLp <= MAXVELOCITY and
                    -MAXVELOCITY <= vRp <= MAXVELOCITY):
                continue
            if vLp < 0 and vRp < 0:   # no reversing
                continue
            
            xp, yp, _ = predict_position(vLp, vRp, robot_x, robot_y, robot_theta, TAU)
            xm, ym, _ = predict_position(vLp, vRp, robot_x, robot_y, robot_theta, TAU/2)

            dist_obs = min(closest_obstacle_dist(xp, yp),
                           closest_obstacle_dist(xm, ym))
            if dist_obs < 0:
                continue

            d_prev = math.hypot(robot_x - TARGET_X, robot_y - TARGET_Y)
            d_new  = math.hypot(xp      - TARGET_X, yp      - TARGET_Y)
            fwd_benefit = FORWARD_WEIGHT * (d_prev - d_new)
            obs_cost = (OBSTACLE_WEIGHT * (SAFEDIST - dist_obs)
                        if dist_obs < SAFEDIST else 0.0)

            benefit = fwd_benefit - obs_cost
            if benefit > best_benefit:
                best_benefit = benefit
                vL_best, vR_best = vLp, vRp

    vL, vR = vL_best, vR_best


def set_motors(vL_ms, vR_ms):
    BP.set_motor_dps(MOTOR_LEFT,  int(mps_to_dps(vL_ms)))
    BP.set_motor_dps(MOTOR_RIGHT, int(mps_to_dps(vR_ms)))
    
def prune_obstacles():
    """Remove obstacles that are now behind the robot."""
    global obstacles
    obstacles = [obs for obs in obstacles
                 if (obs[0] - robot_x) * math.cos(robot_theta) +
                    (obs[1] - robot_y) * math.sin(robot_theta) > -0.3]

def victory_celebration():
    BP.set_motor_dps(MOTOR_LEFT,  0)
    BP.set_motor_dps(MOTOR_RIGHT, 0)
    time.sleep(0.3)

    # Spin right
    BP.set_motor_dps(MOTOR_LEFT,  400)
    BP.set_motor_dps(MOTOR_RIGHT, -400)
    time.sleep(0.6)

    # Spin left
    BP.set_motor_dps(MOTOR_LEFT,  -400)
    BP.set_motor_dps(MOTOR_RIGHT,  400)
    time.sleep(0.6)

    # Spin right full circle
    BP.set_motor_dps(MOTOR_LEFT,  400)
    BP.set_motor_dps(MOTOR_RIGHT, -400)
    time.sleep(1.2)

    # Stop
    BP.set_motor_dps(MOTOR_LEFT,  0)
    BP.set_motor_dps(MOTOR_RIGHT, 0)
    # ─────────────────────────────────────────────────
    
    
# ─── Main Loop ────────────────────────────────────────────────────────────────
print("DWA navigation starting. Starting...")
time.sleep(0.1)

start_time = time.time()
print("GO!")
printed = 0
try:
    while True:
        t0 = time.time()

        update_odometry()       # 1. odometry from encoders
        sense_obstacles()       # 2. camera -> world-frame obstacle list
        prune_obstacles()
        dwa_step()              # 3. DWA: choose best (vL, vR)
        set_motors(vL, vR)      # 4. command motors

        elapsed = time.time() - start_time
        print(f"t={elapsed:.1f}s | "
              f"pos=({robot_x:.2f},{robot_y:.2f}) | "
              f"hdg={math.degrees(robot_theta):.1f}° | "
              f"vL={vL:.2f} vR={vR:.2f} | "
              f"obs={len(obstacles)}")
        if robot_x >= 3 and not printed:
            print(f">>> Crossed 3m line in {elapsed:.2f}s!")
            printed = 1
            
        if robot_x >= 3.25:
            print(f">>> Crossed 3.25m line in {elapsed:.2f}s!")
            victory_celebration()
            break

        # Sleep remaining DT
        sleep_t = DT - (time.time() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)

except KeyboardInterrupt:
    print("Stopped.")

finally:
    BP.set_motor_dps(MOTOR_LEFT,  0)
    BP.set_motor_dps(MOTOR_RIGHT, 0)
    picam2.stop()
    BP.reset_all()
    print("Done.")
