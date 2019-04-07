import sys
sys.path.append('/usr/local/lib/python3.7/site-packages')

import serial
import cv2
import time
import numpy as np
import math
import botshot_retro
import subprocess

FRAME_SIZE_X = 320
FRAME_SIZE_Y = 240
DEVICE = 0
ONLINE = True
IN_TO_MM = 25.4

def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    return 0

def pt_distance(A, B):
    return ((A[0] - B[0])**2 + (A[1] - B[1])**2)**0.5

def dist_from_height(height):
    theta_y = 75 / FRAME_SIZE_X * height
    return abs(18 / math.tan(math.radians(theta_y)))

def pose(r1, r2):
    x = (r2**2 - r1**2) / 48
    y = 15 + (r2**2 - x**2 - 24*x - 144)**0.5
    return x,y


def find_corner_points(contour):
    M = cv2.moments(contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    # Filter points by where they are relative to the center
    TL_pts = list(filter(lambda C: C[0][0] < cx and C[0][1] < cy, contour))
    TR_pts = list(filter(lambda C: C[0][0] > cx and C[0][1] < cy, contour))
    BR_pts = list(filter(lambda C: C[0][0] > cx and C[0][1] > cy, contour))
    BL_pts = list(filter(lambda C: C[0][0] < cx and C[0][1] > cy, contour))
    if min(len(TL_pts), len(TR_pts), len(BR_pts), len(BL_pts)) == 0:
        return None
    # Categorize the "corner point" by being farthest from the center
    farthest = lambda C: (C[0][0]-cx)**2 + (C[0][1]-cy)**2
    tl = list(sorted(TL_pts, key=farthest, reverse=True))[0]
    tr = list(sorted(TR_pts, key=farthest, reverse=True))[0]
    br = list(sorted(BR_pts, key=farthest, reverse=True))[0]
    bl = list(sorted(BL_pts, key=farthest, reverse=True))[0]
    return np.array([tl, tr, br, bl])
    

# todo get new distortion values
dist = np.array([-0.32589 ,  0.13978  , 0.04418 ,  0.00808,  0.00000],dtype=np.float)
mtx = np.array([[322.40450,  0.   ,      164.84064],
 [  0.    ,     295.58822  ,-1.40328 ],
 [  0.    ,       0.    ,       1.        ]]
,dtype=np.float)
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (320,240), 1, (FRAME_SIZE_X,FRAME_SIZE_Y))

axis = np.float32([[12,0,0], [0,12,0], [0,0,12]]).reshape(-1,3)
cap = cv2.VideoCapture("v4l2src device=\"/dev/video{DEVICE}\" ! video/x-raw, width={FRAME_SIZE_X}, height={FRAME_SIZE_Y} ! videoconvert ! appsink".format(DEVICE=DEVICE, FRAME_SIZE_X=FRAME_SIZE_X, FRAME_SIZE_Y=FRAME_SIZE_Y))
#cap = cv2.VideoCapture("http://frcvision.local:1181/stream.mjpg");
pipeline = botshot_retro.GripPipeline()
v4l2_cmd = ["v4l2-ctl","-d", "{}".format(DEVICE), "--set-ctrl", "brightness=0,contrast=32,saturation=64,hue=90,white_balance_automatic=0,auto_exposure=1,exposure=0,gain_automatic=0,gain=60,sharpness=63"]
# Dirty hack to make sure all settings are applied
subprocess.run(v4l2_cmd)
subprocess.run(v4l2_cmd)

if ONLINE:
    conn = serial.Serial("/dev/serial0", baudrate=115200, timeout=0.0, write_timeout=0)

while True:
    ret, frame = cap.read()

    pipeline.process(frame)
    contours = pipeline.filter_contours_output
    if len(contours) > 0:
        # Take largest contour
        largest_contour = sorted(contours, key=lambda C: cv2.contourArea(C), reverse=True)[0]
        x,y,w,h = cv2.boundingRect(largest_contour)
        corner_pts = find_corner_points(largest_contour)
        if corner_pts is not None:
            #cv2.drawContours(frame, [np.int0(corner_pts)], -1, (0,255,0), 2)
            top_left = corner_pts[0].tolist()
            bottom_left = corner_pts[3].tolist()
            top_right = corner_pts[1].tolist()
            bottom_right = corner_pts[2].tolist()
            # Draw points on frame
            cv2.circle(frame, tuple(top_left[0]), 3, (255,0,0), -1)
            cv2.circle(frame, tuple(bottom_left[0]), 3, (255,255,0), -1)
            cv2.circle(frame, tuple(top_right[0]), 3, (0,0,255), -1)
            cv2.circle(frame, tuple(bottom_right[0]), 3, (0,255,255), -1)


            dx = (top_left[0][0] + top_right[0][0])/2 - FRAME_SIZE_X / 2  # dx is position relative to center
            dir_ = sign(dx)
            dx = abs(dx)
            ratio = dx / (FRAME_SIZE_X / 2)
            angle_diff = math.atan(ratio * math.tan(math.radians(75/2)))
            angle = math.degrees(angle_diff)

            px_dist_left = pt_distance(top_left[0], bottom_left[0])
            px_dist_right = pt_distance(top_right[0], bottom_right[0])
            dist = dist_from_height(max(px_dist_left, px_dist_right))
            ratio = px_dist_left / px_dist_right

            # good ol pythagoras
            y = 12*10-35
            dist = (dist**2 - y**2)**0.5
            if not np.isreal(dist):
                continue
            
            print("dist {} angle {}".format(dist, angle))

            # Draw centerline
            cv2.line(frame, (FRAME_SIZE_X//2, 0), (FRAME_SIZE_X//2, FRAME_SIZE_Y), (0,0,255), 1)
            # Print to serial
            if ONLINE:
                message = b"\x00\x00\x00\x00"
                message += int(angle*10).to_bytes(2, 'little', signed=True)
                message += int(dist).to_bytes(2, 'little', signed=False)
                conn.write(message)
                
    if not ONLINE:
        cv2.imshow("frame", frame)
        cv2.waitKey(10)
