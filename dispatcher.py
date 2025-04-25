import asyncio
import detection
import kml

def task_dispatcher(detector, conn):

    async def wait_for_msg(msg_type):
        while True:
            msg = conn.recv_match(type=msg_type, blocking=False)
            if msg:
                return msg
            await asyncio.sleep(0.05)

    while True:
        rc = wait_for_msg('RC_CHANNELS')

        if (rc.chan10_raw < 1300):
            # idle
            pass
        elif (rc.chan10_raw > 1700):
            kml.kml_creation_task()
        else:
            detection.detection_task(detector, conn)
        