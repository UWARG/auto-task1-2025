import asyncio
import copy
import cv2
import globals
import math
import numpy as np
import pymap3d as pm


# matrix for camera marked 'a'
CAM_MATRIX = np.array([[1665.11573,          0, 1128.21945],
                       [         0, 1652.30659, 622.594934],
                       [         0,          0,          1]], dtype=np.float32)


def detection_task(attitude, position, detector):
    # === grab latest camera frame ===
    with globals.frame_lock:
        frame_ = copy.deepcopy(globals.frame)
    if frame_ is None:
        return
    
    # === perform blob detection ===
    gray = cv2.cvtColor(frame_, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    _, binary = cv2.threshold(blur, 127, 255, cv2.THRESH_BINARY)
    detections_ = detector.detect(binary)

    # === write detections back to mark on preview ===
    with globals.detections_lock:
        # should be a deepcopy, but cv2.keypoint is not pickleable
        # leave for now
        globals.detections = detections_
    
    # === prep transformation matricies ===
    # cam to body
    R_cam_to_body = np.array([[0, -1, 0],
                              [1,  0, 0],
                              [0,  0, 1]], dtype=np.float32)

    # body to ned
    cr, sr = math.cos(attitude.roll), math.sin(attitude.roll)
    cp, sp = math.cos(attitude.pitch), math.sin(attitude.pitch)
    cy, sy = math.cos(attitude.yaw), math.sin(attitude.yaw)
    R_body_to_ned = np.array([[           cp*cy,             cp*sy,      -sp],
                              [sr*sp*cy - cr*sy,  sr*sp*sy + cr*cy,    sr*cp],
                              [cr*sp*cy + sr*sy,  cr*sp*sy - sr*cy,    cr*cp]], dtype=np.float32)

    # ned to ecef
    sin_lat, cos_lat = math.sin(math.radians(position.lat / 1e7)), math.cos(math.radians(position.lat / 1e7))
    sin_lon, cos_lon = math.sin(math.radians(position.lon / 1e7)), math.cos(math.radians(position.lon / 1e7))
    R_ned_to_ecef = np.array([[-sin_lon*cos_lat, -sin_lon*sin_lat,  cos_lon],
                              [ cos_lon*cos_lat,  cos_lon*sin_lat,  sin_lon],
                              [        -sin_lat,          cos_lat,        0]], dtype=np.float32)

    for kp in detections_:
        # normalized camera direction matrix
        Q_cam = np.linalg.inv(CAM_MATRIX) @ np.array([kp.pt[0], kp.pt[1], 1], dtype=np.float32).T
        
        # rotate camera --> body --> ned
        Q_ned = R_body_to_ned @ R_cam_to_body @ Q_cam.T
        
        # scale based on altitude reading
        P_ned = Q_ned * (position.relative_alt / 1e3) / Q_ned[2]
        
        # rotate to ECEF
        P_ecef = R_ned_to_ecef @ P_ned.T
        
        # convert GPS from LLA to ECEF
        drone_x, drone_y, drone_z = pm.geodetic2ecef(position.lat / 1e7, position.lon / 1e7, position.alt / 10e3)

        # add drone vector and POI vector
        poi_lat, poi_lon, _ = pm.ecef2geodetic(drone_x + P_ecef[0], drone_y + P_ecef[1], drone_z + P_ecef[2])
        globals.poi_list.append((poi_lat, poi_lon, 0))
        globals.new_poi = True
