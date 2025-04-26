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
        rc, attitude, position = await asyncio.gather(
            wait_for_msg('RC_CHANNELS'),
            wait_for_msg('ATTITUDE'),
            wait_for_msg('GLOBAL_POSITION_INT')
        )
        return (rc, attitude, position)
        
    while True:
        rc, attitude, position = asyncio.run(get_drone_telemetry())

        if (rc.chan10_raw < 1200):
            # idle
            pass
        elif (rc.chan10_raw > 1700):
            # generate kml file
            kml.kml_creation_task(conn)
        else:
            # detect hotspots
            detection.detection_task(attitude, position, detector)
        