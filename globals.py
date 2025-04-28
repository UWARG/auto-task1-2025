import threading


DIST_ERROR = 0.0005
MIN_POINTS_PER_CLUSTER = 3
KML_SENDING_DELAY = 10


frame_lock = threading.Lock()
frame = None

detections_lock = threading.Lock()
detections = ()

new_poi = False
poi_list = []

cluster_list = []
points_in_cluster = []
last_kml_send_time = 0
