from picamera2 import Picamera2
from pymavlink import mavutil
import copy
import cv2
import dispatcher
import globals
import threading


def init_camera():
    cam = Picamera2()
    
    cam_res = {"size": (2304,1296)}
    cam_controls = {"AnalogueGain": 8.0, "AfMode": 0, "LensPosition": 0.0, "ExposureTime": 10}
    
    cam_cfg = cam.create_still_configuration(main=cam_res, controls=cam_controls)
    cam.configure(cam_cfg)
    cam.start()
    
    return cam
    

def init_detector():
    params = cv2.SimpleBlobDetector_Params()

    params.filterByArea = True
    params.minArea = 300
    params.maxArea = 6000

    params.filterByCircularity = False
    #params.minCircularity = 0.8
    #params.maxCircularity = 1

    params.filterByColor = True
    params.blobColor = 255

    params.filterByInertia = False
    #params.minInertiaRatio = 0.7
    #params.maxInertiaRatio = 1

    params.filterByConvexity = False
    #params.minConvexity = 0.7
    #params.maxConvexity = 1

    return cv2.SimpleBlobDetector_create(params)
    

def init_mavlink():
    conn = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    print('Waiting for Mavlink connection on UART...')
    conn.wait_heartbeat()
    print('Mavlink heartbeat received!')
    
    test = conn.recv_match(type='ATTITUDE', blocking=True, timeout=3)
    if not test:
        print('No ATTITUDE message received. Requesting...')
        msg = conn.mav.command_long_encode(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            250000,
            0,
            0,
            0,
            0,
            0
        )
        conn.mav.send(msg)
    else:
        print('ATTITUDE message received!')
    
    test = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
    if not test:
        print('No GLOBAL_POSITION_INT message received. Requesting...')
        msg = conn.mav.command_long_encode(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            250000,
            0,
            0,
            0,
            0,
            0
        )
        conn.mav.send(msg)
    else:
        print('GLOBAL_POSITION_INT message received!')
    
    return conn


def main():
    global frame
    
    cam      = init_camera()
    detector = init_detector()
    conn     = init_mavlink()
    
    tasks = threading.Thread(target=dispatcher.task_dispatcher, args=(detector,conn), daemon=True)
    tasks.start()
    
    while True:
        frame_ = cam.capture_array()
        
        with globals.frame_lock:
            frame = copy.deepcopy(frame_)
            
        with globals.detections_lock:
            # should be a deepcopy, but cv2.keypoint is not pickleable
            # leave for now
            detections_ = globals.detections
            
        for kp in detections_:
            x = int(kp.pt[0])
            y = int(kp.pt[1])
            size = int(kp.size) * 3
            top_left = (x - size // 2, y - size // 2)
            bottom_right = (x + size // 2, y + size // 2)
            cv2.rectangle(frame_, top_left, bottom_right, (0,255,0), 3)
        
        cv2.imshow('preview', cv2.cvtColor(cv2.resize(frame_, (640,360)), cv2.COLOR_RGB2BGR))
        
        cv2.waitKey(1)
        
        
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCTRL+C received. Exiting...')
