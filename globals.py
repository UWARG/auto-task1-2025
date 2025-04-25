import threading

frame_lock = threading.Lock()
frame = None

detections_lock = threading.Lock()
detections = ()

new_poi = False
poi_list = []

cluster_list = []
points_in_cluster = []
