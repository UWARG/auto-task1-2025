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
    
    async def get_drone_telemetry():
        rc = await wait_for_msg('RC_CHANNELS')
        return (rc)
        
    

    while True:
        rc = asyncio.run(get_drone_telemetry())

        if (rc.chan10_raw < 1200):
            # idle
            pass
        elif (rc.chan10_raw > 1700):
            kml.kml_creation_task(conn)
        else:
            detection.detection_task(detector, conn)
        