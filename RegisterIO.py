# /* ============================================
# navX MXP source code is placed under the MIT license
# Copyright (c) 2015 Kauai Labs

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# ===============================================
#  */

import machine
import time
from Registers import *

MAX_WPILIB_I2C_READ_BYTES = 127
NUM_IGNORED_SUCCESSIVE_ERRORS = 50
NAVX_ADDR = 0x32

class RegisterIO:
    """Provies functions to read and write from registers"""

    def __init__(self, update_rate_hz):
        self.successive_error_count = 0
        self.port = machine.I2C(1, scl=machine.Pin(3), sda=machine.Pin(2), freq=400000)
        self.update_rate_hz = update_rate_hz
        self.sensor_timestamp = 0
        self.board_id = {
            "type": 0,
            "hw_rev": 0,
            "fw_ver_major": 0,
            "fw_ver_minor": 0,
            "fw_revision": 0,
            "unique_id": []
        }
        self.board_state={
            "op_status":0,
            "sensor_status":0,
            "cal_status":0,
            "selftest_status":0,
            "capability_flags":0,
            "update_rate_hz":0,
            "accel_fsr_g":0,
            "gyro_fsr_dps":0
        }
        self.ahrs_update = {
            "yaw":0,
            "pitch":0,
            "roll":0,
            "compass_heading":0,
            "altitude":0,
            "fused_heading":0,
            "linear_accel_x":0,
            "linear_accel_y":0,
            "linear_accel_z":0,  	
            "mpu_temp":0,
            "quat_w":0,
            "quat_x":0,
            "quat_y":0,
            "quat_z":0,
            "barometric_pressure":0,
            "baro_temp":0,
            "op_status":0,
            "sensor_status":0,
            "cal_status":0,
            "selftest_status":0,	
            "cal_mag_x":0,
            "cal_mag_y":0,
            "cal_mag_z":0,
            "mag_field_norm_ratio":0,
            "mag_field_norm_scalar":0,
            "raw_mag_x":0,
            "raw_mag_y":0,
            "raw_mag_z":0
        }
        self.ahrspos_update = {
            "yaw":0,
            "pitch":0,
            "roll":0,
            "compass_heading":0,
            "altitude":0,
            "fused_heading":0,
            "linear_accel_x":0,
            "linear_accel_y":0,
            "linear_accel_z":0,  	
            "mpu_temp":0,
            "quat_w":0,
            "quat_x":0,
            "quat_y":0,
            "quat_z":0,
            "barometric_pressure":0,
            "baro_temp":0,
            "op_status":0,
            "sensor_status":0,
            "cal_status":0,
            "selftest_status":0,	
            "vel_x":0,
            "vel_y":0,
            "vel_z":0,
            "disp_x":0,
            "disp_y":0,
            "disp_z":0
        }
        self.raw_data_update = {
            "gyro_x":0,
            "gyro_y":0,
            "gyro_z":0,
            "accel_x":0,
            "accel_y":0,
            "accel_z":0,
            "mag_x":0,
            "mag_y":0,
            "mag_z":0,
            "temp_c":0
        }

    def IsDisplacementSupported(self):
        return True if ((self.board_state['capability_flags'] & NAVX_CAPABILITY_FLAG_VEL_AND_DISP) != 0) else False

    def read(self, first_address, buffer: bytearray):
        len1 = 0
        len1 = len(buffer)
        buffer_offset = 0
        read_buffer = bytearray(MAX_WPILIB_I2C_READ_BYTES)
        while (len1 > 0):
            read_len1 = MAX_WPILIB_I2C_READ_BYTES if (len1 > MAX_WPILIB_I2C_READ_BYTES) else len1
            good = True
            try:
                self.port.writeto_mem(NAVX_ADDR, first_address + buffer_offset, bytes([read_len1]))
            except Exception as e:
                print(e)
                good = False
            try:
                self.port.readfrom_into(NAVX_ADDR, read_buffer)
            except Exception as e:
                print(e)
                good = False
            if good:
                for i in range(read_len1):
                    buffer[buffer_offset+i] = read_buffer[i]
                buffer_offset += read_len1
                len1 -= read_len1
                self.successive_error_count = 0
            else:
                self.successive_error_count+=1
                if (self.successive_error_count % NUM_IGNORED_SUCCESSIVE_ERRORS == 1):
                    break
                return False
            
        return (len1 == 0)
    
    def write(self, address, value ):
        self.port.writeto_mem(NAVX_ADDR, address | 0x80, bytes([value]))

    def ZeroYaw(self):
        self.write(NAVX_REG_INTEGRATION_CTL, NAVX_INTEGRATION_CTL_RESET_YAW)

    def ZeroDisplacement(self):
        self.write(NAVX_REG_INTEGRATION_CTL,
                        NAVX_INTEGRATION_CTL_RESET_DISP_X |
                        NAVX_INTEGRATION_CTL_RESET_DISP_Y |
                        NAVX_INTEGRATION_CTL_RESET_DISP_Z )

    def GetConfiguration(self):
        IMURegisters_i = IMURegisters()
        success = False
        retry_count = 0
        while retry_count < 3 and not success:
            config = bytearray(NAVX_REG_SENSOR_STATUS_H+1)
            if self.read(NAVX_REG_WHOAMI, config) and (config[NAVX_REG_WHOAMI] == NAVX_MODEL_NAVX_MXP):
                self.board_id['hw_rev']                 = config[NAVX_REG_HW_REV]
                self.board_id['fw_ver_major']           = config[NAVX_REG_FW_VER_MAJOR]
                self.board_id['fw_ver_minor']           = config[NAVX_REG_FW_VER_MINOR]
                self.board_id['type']                   = config[NAVX_REG_WHOAMI]
    
                self.board_state['cal_status']          = config[NAVX_REG_CAL_STATUS]
                self.board_state['op_status']           = config[NAVX_REG_OP_STATUS]
                self.board_state['selftest_status']     = config[NAVX_REG_SELFTEST_STATUS]
                self.board_state['sensor_status']       = IMURegisters_i.decodeProtocolUint16(config, NAVX_REG_SENSOR_STATUS_L)
                self.board_state['gyro_fsr_dps']        = IMURegisters_i.decodeProtocolUint16(config, NAVX_REG_GYRO_FSR_DPS_L)
                self.board_state['accel_fsr_g']         = config[NAVX_REG_ACCEL_FSR_G]
                self.board_state['update_rate_hz']      = config[NAVX_REG_UPDATE_RATE_HZ]
                self.board_state['capability_flags']    = IMURegisters_i.decodeProtocolUint16(config, NAVX_REG_CAPABILITY_FLAGS_L)
                success = True
            else:
                success = False
                time.sleep(0.05)
        
            retry_count+=1
        return success
    
    def GetCurrentData(self):
        IMURegisters_i = IMURegisters()
        first_address = NAVX_REG_UPDATE_RATE_HZ
        displacement_registers = self.IsDisplacementSupported()
        curr_data = bytearray(NAVX_REG_LAST + 1)
        
        if self.read(first_address, curr_data):
            self.sensor_timestamp = IMURegisters_i.decodeProtocolUint32(curr_data ,NAVX_REG_TIMESTAMP_L_L-first_address)
            self.ahrspos_update['op_status'] = curr_data[NAVX_REG_OP_STATUS - first_address]
            self.ahrspos_update['selftest_status'] = curr_data[NAVX_REG_SELFTEST_STATUS - first_address]
            self.ahrspos_update['cal_status'] = curr_data[NAVX_REG_CAL_STATUS - first_address]
            self.ahrspos_update['sensor_status'] = curr_data[NAVX_REG_SENSOR_STATUS_L - first_address]
            self.ahrspos_update['yaw'] = IMURegisters_i.decodeProtocolSignedHundredthsFloat(curr_data, NAVX_REG_YAW_L-first_address)
            self.ahrspos_update['pitch'] = IMURegisters_i.decodeProtocolSignedHundredthsFloat(curr_data, NAVX_REG_PITCH_L-first_address)
            self.ahrspos_update['roll'] = IMURegisters_i.decodeProtocolSignedHundredthsFloat(curr_data, NAVX_REG_ROLL_L-first_address)
            self.ahrspos_update['compass_heading'] = IMURegisters_i.decodeProtocolUnsignedHundredthsFloat(curr_data, NAVX_REG_HEADING_L-first_address)
            self.ahrspos_update['mpu_temp'] = IMURegisters_i.decodeProtocolSignedHundredthsFloat(curr_data, NAVX_REG_MPU_TEMP_C_L - first_address)
            self.ahrspos_update['linear_accel_x'] = IMURegisters_i.decodeProtocolSignedThousandthsFloat(curr_data, NAVX_REG_LINEAR_ACC_X_L-first_address)
            self.ahrspos_update['linear_accel_y'] = IMURegisters_i.decodeProtocolSignedThousandthsFloat(curr_data , NAVX_REG_LINEAR_ACC_Y_L-first_address)
            self.ahrspos_update['linear_accel_z'] = IMURegisters_i.decodeProtocolSignedThousandthsFloat(curr_data , NAVX_REG_LINEAR_ACC_Z_L-first_address)
            self.ahrspos_update['altitude'] = IMURegisters_i.decodeProtocol1616Float(curr_data , NAVX_REG_ALTITUDE_D_L - first_address)
            self.ahrspos_update['barometric_pressure'] = IMURegisters_i.decodeProtocol1616Float(curr_data , NAVX_REG_PRESSURE_DL - first_address)
            self.ahrspos_update['fused_heading'] = IMURegisters_i.decodeProtocolUnsignedHundredthsFloat(curr_data , NAVX_REG_FUSED_HEADING_L-first_address)
            self.ahrspos_update['quat_w'] = (IMURegisters_i.decodeProtocolInt16(curr_data , NAVX_REG_QUAT_W_L-first_address)) / 32768.0
            self.ahrspos_update['quat_x'] = (IMURegisters_i.decodeProtocolInt16(curr_data , NAVX_REG_QUAT_X_L-first_address)) / 32768.0
            self.ahrspos_update['quat_y'] = (IMURegisters_i.decodeProtocolInt16(curr_data , NAVX_REG_QUAT_Y_L-first_address)) / 32768.0
            self.ahrspos_update['quat_z'] = (IMURegisters_i.decodeProtocolInt16(curr_data , NAVX_REG_QUAT_Z_L-first_address)) / 32768.0
            if displacement_registers:
                self.ahrspos_update['vel_x']      = IMURegisters_i.decodeProtocol1616Float(curr_data , NAVX_REG_VEL_X_I_L-first_address)
                self.ahrspos_update['vel_y']      = IMURegisters_i.decodeProtocol1616Float(curr_data , NAVX_REG_VEL_Y_I_L-first_address)
                self.ahrspos_update['vel_z']      = IMURegisters_i.decodeProtocol1616Float(curr_data , NAVX_REG_VEL_Z_I_L-first_address)
                self.ahrspos_update['disp_x']      = IMURegisters_i.decodeProtocol1616Float(curr_data , NAVX_REG_DISP_X_I_L-first_address)
                self.ahrspos_update['disp_y']      = IMURegisters_i.decodeProtocol1616Float(curr_data , NAVX_REG_DISP_Y_I_L-first_address)
                self.ahrspos_update['disp_z']      = IMURegisters_i.decodeProtocol1616Float(curr_data , NAVX_REG_DISP_Z_I_L-first_address)
            else:
                self.ahrs_update['op_status']           = self.ahrspos_update['op_status']
                self.ahrs_update['selftest_status']     = self.ahrspos_update['selftest_status']
                self.ahrs_update['cal_status']          = self.ahrspos_update['cal_status']
                self.ahrs_update['sensor_status']       = self.ahrspos_update['sensor_status']
                self.ahrs_update['yaw']                 = self.ahrspos_update['yaw']
                self.ahrs_update['pitch']               = self.ahrspos_update['pitch']
                self.ahrs_update['roll']                = self.ahrspos_update['roll']
                self.ahrs_update['compass_heading']     = self.ahrspos_update['compass_heading']
                self.ahrs_update['mpu_temp']            = self.ahrspos_update['mpu_temp']
                self.ahrs_update['linear_accel_x']      = self.ahrspos_update['linear_accel_x']
                self.ahrs_update['linear_accel_y']      = self.ahrspos_update['linear_accel_y']
                self.ahrs_update['linear_accel_z']      = self.ahrspos_update['linear_accel_z']
                self.ahrs_update['altitude']            = self.ahrspos_update['altitude']
                self.ahrs_update['barometric_pressure'] = self.ahrspos_update['barometric_pressure']
                self.ahrs_update['fused_heading']       = self.ahrspos_update['fused_heading']

            self.board_state['update_rate_hz']  = curr_data[NAVX_REG_UPDATE_RATE_HZ-first_address]
            self.board_state['gyro_fsr_dps']    = IMURegisters_i.decodeProtocolUint16(curr_data , NAVX_REG_GYRO_FSR_DPS_L-first_address)
            self.board_state['accel_fsr_g']     = curr_data[NAVX_REG_ACCEL_FSR_G-first_address]
            self.board_state['capability_flags']= IMURegisters_i.decodeProtocolUint16(curr_data , NAVX_REG_CAPABILITY_FLAGS_L-first_address)

            self.raw_data_update['gyro_x']      = IMURegisters_i.decodeProtocolInt16(curr_data ,  NAVX_REG_GYRO_X_L-first_address)
            self.raw_data_update['gyro_y']      = IMURegisters_i.decodeProtocolInt16(curr_data ,  NAVX_REG_GYRO_Y_L-first_address)
            self.raw_data_update['gyro_z']      = IMURegisters_i.decodeProtocolInt16(curr_data ,  NAVX_REG_GYRO_Z_L-first_address)
            self.raw_data_update['accel_x']     = IMURegisters_i.decodeProtocolInt16(curr_data ,  NAVX_REG_ACC_X_L-first_address)
            self.raw_data_update['accel_y']     = IMURegisters_i.decodeProtocolInt16(curr_data ,  NAVX_REG_ACC_Y_L-first_address)
            self.raw_data_update['accel_z']     = IMURegisters_i.decodeProtocolInt16(curr_data ,  NAVX_REG_ACC_Z_L-first_address)
            self.raw_data_update['mag_x']       = IMURegisters_i.decodeProtocolInt16(curr_data ,  NAVX_REG_MAG_X_L-first_address)
            self.raw_data_update['temp_c']      = self.ahrspos_update['mpu_temp']