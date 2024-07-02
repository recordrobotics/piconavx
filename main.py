import sys
import machine
from mpy_env import get_env
from RegisterIO import *
from host_protocol import *
import errno
import uasyncio
import gc
import time

# https://forum.micropython.org/viewtopic.php?t=3499
# Returns (used, free, total)
def mem_stats(force_gc:bool = False):
    if force_gc:
        gc.collect()
        
    F = gc.mem_free()
    A = gc.mem_alloc()
    T = F+A
    return (A,F,T)

ahrs_update={}
ahrspos_update={}
raw_data_update={}
board_state={}
board_id={}
health={
    "mem_used":0,
    "mem_total":0
}

health_requested:bool = False
boardstate_requested:bool = False
boardid_requested:bool = False
zeroyaw_requested:bool = False
zerodisp_requested:bool = False

def get_health():
    global health
    
    (used, _, total) = mem_stats(True)
    health['mem_used'] = used
    health['mem_total'] = total

host_update_delay=1
                
async def sensor_poll():
    global ahrs_update
    global ahrspos_update
    global raw_data_update
    global board_state
    global board_id
    global host_update_delay
    global boardstate_requested
    global boardid_requested
    global zeroyaw_requested
    global zerodisp_requested
    
    pin = machine.Pin("LED", machine.Pin.OUT)
    NAVX_DEFAULT_UPDATE_RATE_HZ = 60
    NAV = RegisterIO(NAVX_DEFAULT_UPDATE_RATE_HZ)
    
    ahrs_update = NAV.ahrs_update.copy()
    ahrspos_update = NAV.ahrspos_update.copy()
    raw_data_update = NAV.raw_data_update.copy()
    board_state = NAV.board_state.copy()
    board_id = NAV.board_id.copy()
    
    if NAV.GetConfiguration() is False:
        print("NavX Error: Could not get configuration. Make sure it is connected")
    print("Displacement supported: "+str(NAV.IsDisplacementSupported()))
    
    update_delay = 1.0 / NAV.board_state['update_rate_hz']
    host_update_delay = update_delay
    
    while True:
        try:
            pin.toggle()
            
            if zeroyaw_requested:
                zeroyaw_requested = False
                NAV.ZeroYaw()
            if zerodisp_requested:
                zerodisp_requested = False
                NAV.ZeroDisplacement()
                
            NAV.GetCurrentData()
            ahrs_update = NAV.ahrs_update.copy()
            ahrspos_update = NAV.ahrspos_update.copy()
            raw_data_update = NAV.raw_data_update.copy()
            if boardstate_requested or boardid_requested:
                NAV.GetConfiguration()
                board_state = NAV.board_state.copy()
                board_id = NAV.board_id.copy()
            await uasyncio.sleep(update_delay)
        except Exception as e:
            print("sensor error "+str(type(e)))
            sys.print_exception(e) # type: ignore

async def tcp_client():
    global health_requested
    global boardstate_requested
    global boardid_requested
    
    HOST = get_env('host')
    reader: uasyncio.StreamReader|None = None
    writer: uasyncio.StreamWriter|None = None
    
    updateDataType = SET_DATA_TYPE_AHRSPOS
    
    listener_task:uasyncio.Task|None = None

    while True:
        try:
            if writer is None or reader is None:
                if HOST is not None:
                    try:
                        if listener_task is not None:
                            await listener_task # type: ignore
                            listener_task = None
                        
                        (reader, writer) = await uasyncio.open_connection(HOST['address'], HOST['port']) # type: ignore
                        
                        if writer is not None and reader is not None:
                            # wait for host introduction
                            line:str = ""
                            while (line != HOST_INTRODUCTION):
                                line = (await reader.readline()).decode("ascii") #type: ignore
                                
                            # respond with our id
                            writer.write(format_id(str(get_env('id'))).encode("ascii"))
                            
                            async def host_listener():
                                global updateDataType
                                global health_requested
                                global boardstate_requested
                                global boardid_requested
                                global zeroyaw_requested
                                global zerodisp_requested
                                
                                host_line:str = ""
                                while reader is not None:
                                    try:
                                        host_line = (await reader.readline()).decode("ascii") #type: ignore
                                        command_type = parse_command_type(host_line)
                                        if command_type == COMMAND_TYPE_SET_DATA:
                                            updateDataType = parse_set_data_command(host_line)
                                        elif command_type == COMMAND_TYPE_REQUEST_HEALTH:
                                            health_requested = True
                                        elif command_type == COMMAND_TYPE_REQUEST_BOARDSTATE:
                                            boardstate_requested = True
                                        elif command_type == COMMAND_TYPE_REQUEST_BOARDID:
                                            boardid_requested = True
                                        elif command_type == COMMAND_TYPE_ZERO_YAW:
                                            zeroyaw_requested = True
                                        elif command_type == COMMAND_TYPE_ZERO_DISPLACEMENT:
                                            zerodisp_requested = True
                                    except OSError as e:
                                        if e.errno is errno.ECONNRESET or e.errno is errno.ECONNABORTED or e.errno is errno.ECONNREFUSED:
                                            break
                                        else:
                                            raise e
                            
                            listener_task = uasyncio.create_task(host_listener())
                    except OSError as e:
                        sys.print_exception(e) # type: ignore
                        print("Host not up retrying in 5 seconds...")
                        await uasyncio.sleep(5)
            else:
                try:
                    if updateDataType == SET_DATA_TYPE_RAW:
                        write_raw_update(writer, raw_data_update)
                    elif updateDataType == SET_DATA_TYPE_AHRS:
                        write_ahrs_update(writer, ahrs_update)
                    elif updateDataType == SET_DATA_TYPE_AHRSPOS:
                        write_ahrspos_update(writer, ahrspos_update)
                    elif updateDataType == SET_DATA_TYPE_YPR:
                        write_ypr_update(writer, ahrspos_update)
                        
                    if health_requested:
                        health_requested = False
                        get_health()
                        write_health_update(writer, health)
                    if boardstate_requested:
                        boardstate_requested = False
                        write_boardstate_update(writer, board_state)
                    if boardid_requested:
                        boardid_requested = False
                        write_boardid_update(writer, board_id)
                except OSError as e:
                    if e.errno is errno.ECONNRESET or e.errno is errno.ECONNABORTED or e.errno is errno.ECONNREFUSED:
                        writer.close()
                        reader.close()
                        writer = None
                        reader = None
                    else:
                        raise e
                await uasyncio.sleep(host_update_delay)
        except Exception as e:
            print("tcp error "+str(type(e)))
            sys.print_exception(e) # type: ignore

async def main():
    tasks = []

    # NavX sensor poll task
    m = uasyncio.create_task(sensor_poll())
    await uasyncio.sleep(0)
    tasks.append(m)

    # TCP client task
    s = uasyncio.create_task(tcp_client())
    tasks.append(s)

    await uasyncio.sleep(0)
    for p in tasks:
        await p

try:
    print('starting event loop')
    uasyncio.run(main())

except KeyboardInterrupt:
    print('Interrupted')
except Exception as e:
    print("caught")
    sys.print_exception(e) # type: ignore
finally:
    uasyncio.new_event_loop()
    print('ended event loop')

print('resetting to known state')
time.sleep(2)
machine.reset()