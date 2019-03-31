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

def drawAxes(img, corners, imgpts):
    corner = tuple(np.int0(corners[0].ravel()))
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 2)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 2)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 2)
    return img

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
dist = np.array([-0.04669767 , 0.2858172 , -0.00896493 , 0.01114811, -0.33249969],dtype=np.float)
mtx = np.array([[276.24757534   ,0.,         169.34649796],
 [  0.    ,     276.95154197 ,113.89836015],
 [  0.   ,        0.    ,       1.        ]]
,dtype=np.float)
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (FRAME_SIZE_X,FRAME_SIZE_Y), 1, (FRAME_SIZE_X,FRAME_SIZE_Y))

axis = np.float32([[12,0,0], [0,12,0], [0,0,12]]).reshape(-1,3)
cap = cv2.VideoCapture("v4l2src device=\"/dev/video{DEVICE}\" ! video/x-raw, width={FRAME_SIZE_X}, height={FRAME_SIZE_Y} ! videoconvert ! appsink".format(DEVICE=DEVICE, FRAME_SIZE_X=FRAME_SIZE_X, FRAME_SIZE_Y=FRAME_SIZE_Y))

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
            cv2.drawContours(frame, [np.int0(corner_pts)], -1, (0,255,0), 2)
            top_left = corner_pts[0].tolist()
            bottom_left = corner_pts[3].tolist()
            top_right = corner_pts[1].tolist()
            bottom_right = corner_pts[2].tolist()

            bb_z_dist = 6
            # PnP
            imgpoints = np.array([top_left, bottom_left, top_right, bottom_right], dtype=np.float)
            objpoints = np.array([[-12.,18.,bb_z_dist], [-12.,0.,bb_z_dist], [12.,18.,bb_z_dist], [12, 0, bb_z_dist]], dtype=np.float)
            ret,rvecs, tvecs= cv2.solvePnP(objpoints, imgpoints, mtx, dist)
            x = tvecs[0][0]
            z = tvecs[2][0]
            angle = math.degrees(math.atan2(x,z))
            dist = (x**2 + z**2)**0.5
            print("X-Z distance: {}".format(dist))
            print("Angle: {}".format(angle))
            if ONLINE:
                message = b"\x00\x00\x00\x00"
                message += int(angle*10).to_bytes(2, 'little', signed=True)
                message += int(dist).to_bytes(2, 'little', signed=False)
                conn.write(message)
                
    if not ONLINE:
        cv2.imshow("frame", frame)
        cv2.waitKey(10)
