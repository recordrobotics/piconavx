import sys
import machine
from mpy_env import get_env
from RegisterIO import *
from host_protocol import *
from feed import *
import errno
import uasyncio
import gc
import time

coreTempSensor = machine.ADC(machine.ADC.CORE_TEMP)

def read_core_temp():
    adc_value = coreTempSensor.read_u16()
    volt = (3.3/65535)*adc_value
    temperature = 27 - (volt - 0.706)/0.001721
    return temperature

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
    "mem_total":0,
    "core_temp":0.0
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
    health['core_temp'] = read_core_temp()

host_update_delay=1

updateDataType = SET_DATA_TYPE_AHRSPOS
feed_A = Feed()
feed_B = Feed()
feed_lock_A = uasyncio.Lock()
feed_lock_B = uasyncio.Lock()
feed_switch_lock = uasyncio.Lock()

def reset_state():
    global updateDataType
    global health_requested
    global boardstate_requested
    global boardid_requested
    global zeroyaw_requested
    global zerodisp_requested
    
    updateDataType = SET_DATA_TYPE_AHRSPOS
    health_requested = False
    boardstate_requested = False
    boardid_requested = False
    zeroyaw_requested = False
    zerodisp_requested = False
    
    feed_A.disable()
    feed_B.disable()
    
def submit_feed(mv):
    format_feed_entry(
        ahrspos_update['yaw'],
        ahrspos_update['pitch'],
        ahrspos_update['roll'],
        ahrspos_update['compass_heading'],
        ahrspos_update['altitude'],
        ahrspos_update['fused_heading'],
        ahrspos_update['linear_accel_x'],
        ahrspos_update['linear_accel_y'],
        ahrspos_update['linear_accel_z'],
        ahrspos_update['mpu_temp'],
        ahrspos_update['quat_w'],
        ahrspos_update['quat_x'],
        ahrspos_update['quat_y'],
        ahrspos_update['quat_z'],
        ahrspos_update['barometric_pressure'],
        ahrspos_update['baro_temp'],
        ahrspos_update['vel_x'],
        ahrspos_update['vel_y'],
        ahrspos_update['vel_z'],
        ahrspos_update['disp_x'],
        ahrspos_update['disp_y'],
        ahrspos_update['disp_z'],
        mv, 4
    )

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
    global feed_A
    global feed_B
    
    pin = machine.Pin("LED", machine.Pin.OUT)
    NAVX_DEFAULT_UPDATE_RATE_HZ = 60
    NAV = RegisterIO(NAVX_DEFAULT_UPDATE_RATE_HZ)
    
    ahrs_update = NAV.ahrs_update.copy()
    ahrspos_update = NAV.ahrspos_update.copy()
    raw_data_update = NAV.raw_data_update.copy()
    board_state = NAV.board_state.copy()
    board_id = NAV.board_id.copy()
    
    if NAV.GetConfiguration() is False:
        print("[Error] NavX: Could not get configuration. Make sure it is connected")
    print("[Info] Displacement supported: "+str(NAV.IsDisplacementSupported()))
    
    update_delay = int(1000 / NAV.board_state['update_rate_hz'])
    host_update_delay = update_delay
    
    poll_time = 0
    
    while True:
        poll_time = time.ticks_ms()
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
            
            await feed_switch_lock.acquire() # type: ignore
            if feed_A.accepting():
                await feed_lock_A.acquire() # type: ignore
                try:
                    mv = feed_A.submit(NAV.sensor_timestamp)
                    if mv is not None:
                        submit_feed(mv)
                    else:
                        print("[Error] feed(A) failed to provide valid chunk!")
                finally:
                    feed_lock_A.release()
            elif feed_B.accepting():
                await feed_lock_B.acquire() # type: ignore
                try:
                    mv = feed_B.submit(NAV.sensor_timestamp)
                    if mv is not None:
                        submit_feed(mv)
                    else:
                        print("[Error] feed(B) failed to provide valid chunk!")
                finally:
                    feed_lock_B.release()
            feed_switch_lock.release()
            poll_diff = time.ticks_diff(time.ticks_ms(), poll_time)
            if poll_diff < update_delay:
                await uasyncio.sleep_ms(update_delay - poll_diff)
            else:
                print("[Violation] 'sensor_poll' took "+str(poll_diff)+"ms")
                await uasyncio.sleep_ms(update_delay)
        except Exception as e:
            print("[Error] sensor "+str(type(e)))
            sys.print_exception(e) # type: ignore

async def tcp_client():
    global health_requested
    global boardstate_requested
    global boardid_requested
    global feed_A
    global feed_B
    global updateDataType
    
    HOST = get_env('host')
    reader: uasyncio.StreamReader|None = None
    writer: uasyncio.StreamWriter|None = None
    
    lastFeedSendTime = 0
    
    listener_task:uasyncio.Task|None = None

    while True:
        try:
            if writer is None or reader is None:
                if HOST is not None:
                    try:
                        if listener_task is not None:
                            await listener_task # type: ignore
                            listener_task = None
                        
                        reset_state()
                        gc.collect()
                        lastFeedSendTime = 0
                        
                        (reader, writer) = await uasyncio.open_connection(HOST['address'], HOST['port']) # type: ignore
                        
                        if writer is not None and reader is not None:
                            # wait for host introduction
                            line:str = ""
                            while (line != HOST_INTRODUCTION):
                                line = (await reader.readline()).decode("ascii") #type: ignore
                                
                            # respond with our id
                            writer.write(format_id(str(get_env('id'))).encode("ascii"))
                            await writer.drain() # type: ignore
                            
                            async def host_listener():
                                global updateDataType
                                global health_requested
                                global boardstate_requested
                                global boardid_requested
                                global zeroyaw_requested
                                global zerodisp_requested
                                global feed_A
                                global feed_B
                                
                                host_line:str = ""
                                while reader is not None:
                                    try:
                                        host_line = (await reader.readline()).decode("ascii") #type: ignore
                                        command_type = parse_command_type(host_line)
                                        if command_type == COMMAND_TYPE_SET_DATA:
                                            updateDataType_new = parse_set_data_command(host_line)
                                            if updateDataType_new != updateDataType:
                                                if updateDataType_new == SET_DATA_TYPE_FEED:
                                                    await feed_lock_A.acquire() # type: ignore
                                                    try:
                                                        feed_A.buffer.reset()
                                                        feed_A.enable()
                                                    finally:
                                                        feed_lock_A.release()
                                                elif updateDataType == SET_DATA_TYPE_FEED:
                                                    feed_A.disable()
                                                    feed_B.disable()
                                            updateDataType = updateDataType_new
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
                                        elif command_type == COMMAND_TYPE_SET_FEED_OVERFLOW:
                                            feed_A.feed_overflow = feed_B.feed_overflow = parse_set_feed_overflow_command(host_line)
                                    except OSError as e:
                                        if e.errno is errno.ECONNRESET or e.errno is errno.ECONNABORTED or e.errno is errno.ECONNREFUSED:
                                            break
                                        else:
                                            raise e
                            
                            listener_task = uasyncio.create_task(host_listener())
                    except OSError as e:
                        sys.print_exception(e) # type: ignore
                        print("[Error] Host not up retrying in 5 seconds...")
                        await uasyncio.sleep_ms(5000)
            else:
                try:
                    if updateDataType == SET_DATA_TYPE_RAW:
                        write_raw_update(writer, raw_data_update)
                        await writer.drain() # type: ignore
                    elif updateDataType == SET_DATA_TYPE_AHRS:
                        write_ahrs_update(writer, ahrs_update)
                        await writer.drain() # type: ignore
                    elif updateDataType == SET_DATA_TYPE_AHRSPOS:
                        write_ahrspos_update(writer, ahrspos_update)
                        await writer.drain() # type: ignore
                    elif updateDataType == SET_DATA_TYPE_YPR:
                        write_ypr_update(writer, ahrspos_update)
                        await writer.drain() # type: ignore
                    elif updateDataType == SET_DATA_TYPE_FEED and time.ticks_diff(time.ticks_ms(), lastFeedSendTime) > 3000:
                        lastFeedSendTime = time.ticks_ms()
                        await feed_switch_lock.acquire() # type: ignore
                        if feed_A.accepting():
                            feed_A.disable()
                            feed_B.enable()
                        elif feed_B.accepting():
                            feed_B.disable()
                            feed_A.enable()
                        feed_switch_lock.release()
                        
                        if feed_A.accepting():
                            await feed_lock_B.acquire() # type: ignore
                            try:
                                await write_feed_update(writer, feed_B)
                                feed_B.buffer.reset()
                            finally:
                                feed_lock_B.release()
                        elif feed_B.accepting():
                            await feed_lock_A.acquire() # type: ignore
                            try:
                                await write_feed_update(writer, feed_A)
                                feed_A.buffer.reset()
                            finally:
                                feed_lock_A.release()
                        await uasyncio.sleep_ms(1000)

                    if health_requested:
                        health_requested = False
                        get_health()
                        write_health_update(writer, health)
                        await writer.drain() # type: ignore
                    if boardstate_requested:
                        boardstate_requested = False
                        write_boardstate_update(writer, board_state)
                        await writer.drain() # type: ignore
                    if boardid_requested:
                        boardid_requested = False
                        write_boardid_update(writer, board_id)
                        await writer.drain() # type: ignore
                except OSError as e:
                    if e.errno is errno.ECONNRESET or e.errno is errno.ECONNABORTED or e.errno is errno.ECONNREFUSED:
                        writer.close()
                        reader.close()
                        writer = None
                        reader = None
                    else:
                        raise e
                await uasyncio.sleep_ms(host_update_delay)
        except MemoryError as e:
            sys.print_exception(e) # type: ignore
            (mU,mF,mT) = mem_stats()
            print("[Error] Used "+str(mU)+" / "+str(mT)+ " | "+str(mF))
        except Exception as e:
            print("[Error] tcp "+str(type(e)))
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
    print('[Info] starting event loop')
    uasyncio.run(main())

except KeyboardInterrupt:
    print('[Info] event loop interrupted')
except Exception as e:
    print("[Error] caught error in loop")
    sys.print_exception(e) # type: ignore
finally:
    uasyncio.new_event_loop()
    print('[Error] event loop ended')

print('[Info] resetting to known state')
time.sleep(2)
machine.reset()