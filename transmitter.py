from pymavlink import mavutil
import globals
import math
import time


def cluster_estimation():      
    def get_distance(point1, point2):
        lat_diff = abs(point1[0] - point2[0])
        lon_diff = abs(point1[1] - point2[1])
        return math.sqrt(lat_diff ** 2 + lon_diff ** 2)
    
    cur_point_num = 0

    # Create cluster list
    for cur_point in globals.poi_list:
        # Check point against each current cluster
        cluster_point_num = 0
        in_cluster = False
        for cluster_point in globals.cluster_list:
            dist = get_distance(cur_point, cluster_point)
            if (dist <= globals.DIST_ERROR):
                globals.points_in_cluster[cluster_point_num] += 1
                in_cluster = True

            cluster_point_num += 1

        # If not in any clusters, make new cluster
        if (not in_cluster):
            globals.cluster_list.append(cur_point)
            globals.points_in_cluster.append(1)

        cur_point_num += 1

    # Remove clusters with less than 3 points
    i = 0
    while (i < len(globals.cluster_list)):
        if (globals.points_in_cluster[i] < globals.MIN_POINTS_PER_CLUSTER):
            del globals.cluster_list[i]
            del globals.points_in_cluster[i]
        else:
            i += 1


def transmit_to_ground_station(conn):
    if globals.new_poi:
        cluster_estimation()
        globals.new_poi = False

    now = time.time()
    if now - globals.last_kml_send_time > globals.KML_SENDING_DELAY:
        globals.last_kml_send_time = now
        for cluster in globals.cluster_list:
            severity = mavutil.mavlink.MAV_SEVERITY_NOTICE
            conn.mav.statustext_send(severity, cluster.encode('uft-8'))
