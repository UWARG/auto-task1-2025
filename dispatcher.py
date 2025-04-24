import detection

def task_dispatcher(detector, conn):
    while True:
        detection.detection_task(detector, conn)
        