import uasyncio

HOST_INTRODUCTION = "PICONAVX-HOST\r\n"

HOST_DATA_FIELD_SEPARATOR = '|'

HOST_DATA_TYPE_ID = "ID:"
HOST_DATA_TYPE_RAW = "RAW:"
HOST_DATA_TYPE_AHRS = "AHRS:"
HOST_DATA_TYPE_AHRSPOS = "AHRSPOS:"
HOST_DATA_TYPE_YPR = "YPR:"
HOST_DATA_TYPE_HEALTH = "HEALTH:"
HOST_DATA_TYPE_BOARDSTATE = "BSTATE:"
HOST_DATA_TYPE_BOARDID = "BID:"

HOST_COMMAND_SET_DATA_TYPE = "SETDATA:"
HOST_COMMAND_SET_DATA_TYPE_RAW = "RAW"
HOST_COMMAND_SET_DATA_TYPE_AHRS = "AHRS"
HOST_COMMAND_SET_DATA_TYPE_AHRSPOS = "AHRSPOS"
HOST_COMMAND_SET_DATA_TYPE_YPR = "YPR"

HOST_COMMAND_REQUEST_HEALTH = "GETHEALTH:"
HOST_COMMAND_REQUEST_BOARDSTATE = "GETBSTATE:"
HOST_COMMAND_REQUEST_BOARDID = "GETBID:"
HOST_COMMAND_ZERO_YAW = "ZEROYAW:"
HOST_COMMAND_ZERO_DISPLACEMENT = "ZERODISP:"

COMMAND_TYPE_UNKNOWN = 0
COMMAND_TYPE_SET_DATA = 1
COMMAND_TYPE_REQUEST_HEALTH = 2
COMMAND_TYPE_REQUEST_BOARDSTATE = 3
COMMAND_TYPE_REQUEST_BOARDID = 4
COMMAND_TYPE_ZERO_YAW = 5
COMMAND_TYPE_ZERO_DISPLACEMENT = 6

SET_DATA_TYPE_UNKNOWN = 0
SET_DATA_TYPE_RAW = 1
SET_DATA_TYPE_AHRS = 2
SET_DATA_TYPE_AHRSPOS = 3
SET_DATA_TYPE_YPR = 4

def parse_command_type(line:str):
    if(line.startswith(HOST_COMMAND_SET_DATA_TYPE)):
        return COMMAND_TYPE_SET_DATA
    elif(line.startswith(HOST_COMMAND_REQUEST_HEALTH)):
        return COMMAND_TYPE_REQUEST_HEALTH
    elif(line.startswith(HOST_COMMAND_REQUEST_BOARDSTATE)):
        return COMMAND_TYPE_REQUEST_BOARDSTATE
    elif(line.startswith(HOST_COMMAND_REQUEST_BOARDID)):
        return COMMAND_TYPE_REQUEST_BOARDID
    elif(line.startswith(HOST_COMMAND_ZERO_YAW)):
        return COMMAND_TYPE_ZERO_YAW
    elif(line.startswith(HOST_COMMAND_ZERO_DISPLACEMENT)):
        return COMMAND_TYPE_ZERO_DISPLACEMENT
    else:
        return COMMAND_TYPE_UNKNOWN
    
def parse_set_data_command(line:str):
    line = line[len(HOST_COMMAND_SET_DATA_TYPE):].strip()
    if line == HOST_COMMAND_SET_DATA_TYPE_RAW:
        return SET_DATA_TYPE_RAW
    elif line == HOST_COMMAND_SET_DATA_TYPE_AHRS:
        return SET_DATA_TYPE_AHRS
    elif line == HOST_COMMAND_SET_DATA_TYPE_AHRSPOS:
        return SET_DATA_TYPE_AHRSPOS
    elif line == HOST_COMMAND_SET_DATA_TYPE_YPR:
        return SET_DATA_TYPE_YPR
    else:
        return SET_DATA_TYPE_UNKNOWN
    

def format_id(id:str):
    return f"{HOST_DATA_TYPE_ID}{id}\n"

def format_health_update(memoryUsed:int,memoryTotal:int):
    return f"{HOST_DATA_TYPE_HEALTH}{memoryUsed}{HOST_DATA_FIELD_SEPARATOR}{memoryTotal}\n"

def format_ypr_update(yaw:float,pitch:float,roll:float):
    return f"{HOST_DATA_TYPE_YPR}{yaw}{HOST_DATA_FIELD_SEPARATOR}{pitch}{HOST_DATA_FIELD_SEPARATOR}{roll}\n"

def format_raw_update(
    gyro_x:int,
    gyro_y:int,
    gyro_z:int,
    accel_x:int,
    accel_y:int,
    accel_z:int,
    mag_x:int,
    mag_y:int,
    mag_z:int,
    temp_c:float
    ):
    return f"{HOST_DATA_TYPE_RAW}{gyro_x}{HOST_DATA_FIELD_SEPARATOR}{gyro_y}{HOST_DATA_FIELD_SEPARATOR}{gyro_z}{HOST_DATA_FIELD_SEPARATOR}{accel_x}{HOST_DATA_FIELD_SEPARATOR}{accel_y}{HOST_DATA_FIELD_SEPARATOR}{accel_z}{HOST_DATA_FIELD_SEPARATOR}{mag_x}{HOST_DATA_FIELD_SEPARATOR}{mag_y}{HOST_DATA_FIELD_SEPARATOR}{mag_z}{HOST_DATA_FIELD_SEPARATOR}{temp_c}\n"

def format_ahrspos_update(
    yaw:float,
    pitch:float,
    roll:float,
    compass_heading:float,
    altitude:float,
    fused_heading:float,
    linear_accel_x:float,
    linear_accel_y:float,
    linear_accel_z:float,
    mpu_temp:float,
    quat_w:float,
    quat_x:float,
    quat_y:float,
    quat_z:float,
    barometric_pressure:float,
    baro_temp:float,
    op_status:int,
    sensor_status:int,
    cal_status:int,
    selftest_status:int,
    vel_x:float,
    vel_y:float,
    vel_z:float,
    disp_x:float,
    disp_y:float,
    disp_z:float
    ):
    return f"{HOST_DATA_TYPE_AHRSPOS}{yaw}{HOST_DATA_FIELD_SEPARATOR}{pitch}{HOST_DATA_FIELD_SEPARATOR}{roll}{HOST_DATA_FIELD_SEPARATOR}{compass_heading}{HOST_DATA_FIELD_SEPARATOR}{altitude}{HOST_DATA_FIELD_SEPARATOR}{fused_heading}{HOST_DATA_FIELD_SEPARATOR}{linear_accel_x}{HOST_DATA_FIELD_SEPARATOR}{linear_accel_y}{HOST_DATA_FIELD_SEPARATOR}{linear_accel_z}{HOST_DATA_FIELD_SEPARATOR}{mpu_temp}{HOST_DATA_FIELD_SEPARATOR}{quat_w}{HOST_DATA_FIELD_SEPARATOR}{quat_x}{HOST_DATA_FIELD_SEPARATOR}{quat_y}{HOST_DATA_FIELD_SEPARATOR}{quat_z}{HOST_DATA_FIELD_SEPARATOR}{barometric_pressure}{HOST_DATA_FIELD_SEPARATOR}{baro_temp}{HOST_DATA_FIELD_SEPARATOR}{op_status}{HOST_DATA_FIELD_SEPARATOR}{sensor_status}{HOST_DATA_FIELD_SEPARATOR}{cal_status}{HOST_DATA_FIELD_SEPARATOR}{selftest_status}{HOST_DATA_FIELD_SEPARATOR}{vel_x}{HOST_DATA_FIELD_SEPARATOR}{vel_y}{HOST_DATA_FIELD_SEPARATOR}{vel_z}{HOST_DATA_FIELD_SEPARATOR}{disp_x}{HOST_DATA_FIELD_SEPARATOR}{disp_y}{HOST_DATA_FIELD_SEPARATOR}{disp_z}\n"

def format_ahrs_update(
    yaw:float,
    pitch:float,
    roll:float,
    compass_heading:float,
    altitude:float,
    fused_heading:float,
    linear_accel_x:float,
    linear_accel_y:float,
    linear_accel_z:float,
    mpu_temp:float,
    quat_w:float,
    quat_x:float,
    quat_y:float,
    quat_z:float,
    barometric_pressure:float,
    baro_temp:float,
    op_status:int,
    sensor_status:int,
    cal_status:int,
    selftest_status:int,
    calmag_x:float,
    calmag_y:float,
    calmag_z:float,
    mag_field_norm_ratio:float,
    mag_field_norm_scalar:float,
    rawmag_x:float,
    rawmag_y:float,
    rawmag_z:float
    ):
    return f"{HOST_DATA_TYPE_AHRS}{yaw}{HOST_DATA_FIELD_SEPARATOR}{pitch}{HOST_DATA_FIELD_SEPARATOR}{roll}{HOST_DATA_FIELD_SEPARATOR}{compass_heading}{HOST_DATA_FIELD_SEPARATOR}{altitude}{HOST_DATA_FIELD_SEPARATOR}{fused_heading}{HOST_DATA_FIELD_SEPARATOR}{linear_accel_x}{HOST_DATA_FIELD_SEPARATOR}{linear_accel_y}{HOST_DATA_FIELD_SEPARATOR}{linear_accel_z}{HOST_DATA_FIELD_SEPARATOR}{mpu_temp}{HOST_DATA_FIELD_SEPARATOR}{quat_w}{HOST_DATA_FIELD_SEPARATOR}{quat_x}{HOST_DATA_FIELD_SEPARATOR}{quat_y}{HOST_DATA_FIELD_SEPARATOR}{quat_z}{HOST_DATA_FIELD_SEPARATOR}{barometric_pressure}{HOST_DATA_FIELD_SEPARATOR}{baro_temp}{HOST_DATA_FIELD_SEPARATOR}{op_status}{HOST_DATA_FIELD_SEPARATOR}{sensor_status}{HOST_DATA_FIELD_SEPARATOR}{cal_status}{HOST_DATA_FIELD_SEPARATOR}{selftest_status}{HOST_DATA_FIELD_SEPARATOR}{calmag_x}{HOST_DATA_FIELD_SEPARATOR}{calmag_y}{HOST_DATA_FIELD_SEPARATOR}{calmag_z}{HOST_DATA_FIELD_SEPARATOR}{mag_field_norm_ratio}{HOST_DATA_FIELD_SEPARATOR}{mag_field_norm_scalar}{HOST_DATA_FIELD_SEPARATOR}{rawmag_x}{HOST_DATA_FIELD_SEPARATOR}{rawmag_y}{HOST_DATA_FIELD_SEPARATOR}{rawmag_z}\n"

def format_boardstate_update(
    op_status:int,
    sensor_status:int,
    cal_status:int,
    selftest_status:int,
    capability_flags:int,
    update_rate_hz:int,
    accel_fsr_g:int,
    gyro_fsr_dps:int
    ):
    return f"{HOST_DATA_TYPE_BOARDSTATE}{op_status}{HOST_DATA_FIELD_SEPARATOR}{sensor_status}{HOST_DATA_FIELD_SEPARATOR}{cal_status}{HOST_DATA_FIELD_SEPARATOR}{selftest_status}{HOST_DATA_FIELD_SEPARATOR}{capability_flags}{HOST_DATA_FIELD_SEPARATOR}{update_rate_hz}{HOST_DATA_FIELD_SEPARATOR}{accel_fsr_g}{HOST_DATA_FIELD_SEPARATOR}{gyro_fsr_dps}\n"

def format_boardid_update(
    type:int,
    hw_rev:int,
    fw_ver_major:int,
    fw_ver_minor:int,
    fw_revision:int
    ):
    return f"{HOST_DATA_TYPE_BOARDID}{type}{HOST_DATA_FIELD_SEPARATOR}{hw_rev}{HOST_DATA_FIELD_SEPARATOR}{fw_ver_major}{HOST_DATA_FIELD_SEPARATOR}{fw_ver_minor}{HOST_DATA_FIELD_SEPARATOR}{fw_revision}\n"

def write_health_update(writer: uasyncio.StreamWriter, health):
    writer.write(format_health_update(
        health['mem_used'],
        health['mem_total']
        ).encode("ascii"))

def write_ypr_update(writer: uasyncio.StreamWriter, ahrspos_update):
    writer.write(format_ypr_update(
        ahrspos_update['yaw'],
        ahrspos_update['pitch'],
        ahrspos_update['roll']
        ).encode("ascii"))

def write_raw_update(writer: uasyncio.StreamWriter, raw_data_update):
    writer.write(format_raw_update(
        raw_data_update['gyro_x'],
        raw_data_update['gyro_y'],
        raw_data_update['gyro_z'],
        raw_data_update['accel_x'],
        raw_data_update['accel_y'],
        raw_data_update['accel_z'],
        raw_data_update['mag_x'],
        raw_data_update['mag_y'],
        raw_data_update['mag_z'],
        raw_data_update['temp_c']
        ).encode("ascii"))
    
def write_ahrspos_update(writer: uasyncio.StreamWriter, ahrspos_update):
    writer.write(format_ahrspos_update(
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
        ahrspos_update['op_status'],
        ahrspos_update['sensor_status'],
        ahrspos_update['cal_status'],
        ahrspos_update['selftest_status'],
        ahrspos_update['vel_x'],
        ahrspos_update['vel_y'],
        ahrspos_update['vel_z'],
        ahrspos_update['disp_x'],
        ahrspos_update['disp_y'],
        ahrspos_update['disp_z']
        ).encode("ascii"))
    
def write_ahrs_update(writer: uasyncio.StreamWriter, ahrs_update):
    writer.write(format_ahrs_update(
        ahrs_update['yaw'],
        ahrs_update['pitch'],
        ahrs_update['roll'],
        ahrs_update['compass_heading'],
        ahrs_update['altitude'],
        ahrs_update['fused_heading'],
        ahrs_update['linear_accel_x'],
        ahrs_update['linear_accel_y'],
        ahrs_update['linear_accel_z'],
        ahrs_update['mpu_temp'],
        ahrs_update['quat_w'],
        ahrs_update['quat_x'],
        ahrs_update['quat_y'],
        ahrs_update['quat_z'],
        ahrs_update['barometric_pressure'],
        ahrs_update['baro_temp'],
        ahrs_update['op_status'],
        ahrs_update['sensor_status'],
        ahrs_update['cal_status'],
        ahrs_update['selftest_status'],
        ahrs_update['cal_mag_x'],
        ahrs_update['cal_mag_y'],
        ahrs_update['cal_mag_z'],
        ahrs_update['mag_field_norm_ratio'],
        ahrs_update['mag_field_norm_scalar'],
        ahrs_update['raw_mag_x'],
        ahrs_update['raw_mag_y'],
        ahrs_update['raw_mag_z']
        ).encode("ascii"))
    
def write_boardstate_update(writer: uasyncio.StreamWriter, board_state):
    writer.write(format_boardstate_update(
        board_state['op_status'],
        board_state['sensor_status'],
        board_state['cal_status'],
        board_state['selftest_status'],
        board_state['capability_flags'],
        board_state['update_rate_hz'],
        board_state['accel_fsr_g'],
        board_state['gyro_fsr_dps'],
        ).encode("ascii"))
    
def write_boardid_update(writer: uasyncio.StreamWriter, board_id):
    writer.write(format_boardid_update(
        board_id['type'],
        board_id['hw_rev'],
        board_id['fw_ver_major'],
        board_id['fw_ver_minor'],
        board_id['fw_revision']
        ).encode("ascii"))