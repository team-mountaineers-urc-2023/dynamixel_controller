#!/usr/bin/env python3

from enum import IntEnum
from collections import namedtuple
from typing import List

import dynamixel_sdk as dxl

# TODO write generalized read-from-all function
#      make low-level functions private

# TODO figure out why built-on COMS_SUCCESS constant doesn't work
COMM_SUCCESS = 0
PROTOCOL_VER = 2.0 # code written for protocol 2.0 dynamixels
ENABLE = 1
DISABLE = 0

class OpModes(IntEnum):
    VEL_CONTROL          = 1
    POS_CONTROL          = 3
    EXTENDED_POS_CONTROL = 4
    PWM_CONTROL          = 16

valid_bauds_list = [9600,
                    57600,
                    115200,
                    1000000,
                    2000000,
                    3000000,
                    4000000,
                    4500000]

DXLRegister = namedtuple("DXLRegister", ["addr", "len"])

# TODO as is, this library only works for certain with XL-430-W250-T model dynamixels
# by using multiple register dicts, it could likely be made extensible to other models
reg_dict = {
    "model_num":              DXLRegister(0,2),
    "model_info":             DXLRegister(2,4),
    "firmware_ver":           DXLRegister(6,1),
    "id":                     DXLRegister(7,1),
    "baud_rate":              DXLRegister(8,1),
    "ret_delay_time":         DXLRegister(9,1),
    "drive_mode":             DXLRegister(10,1),
    "op_mode":                DXLRegister(11,1),
    "shadow_id":              DXLRegister(12,1),
    "protocol":               DXLRegister(13,1),
    "homing_offset":          DXLRegister(20,4),
    "moving_thresh":          DXLRegister(24,4),
    "temp_limit":             DXLRegister(31,1),
    "max_volt_limit":         DXLRegister(32,2),
    "min_volt_limit":         DXLRegister(34,2),
    "pwm_limit":              DXLRegister(36,2),
    "vel_limit":              DXLRegister(44,4),
    "max_pos_limit":          DXLRegister(48,4),
    "min_pos_limit":          DXLRegister(52,4),
    "startup_config":         DXLRegister(60,1),
    "shutdown":               DXLRegister(63,1),
    "torque_enable":          DXLRegister(64,1),
    "led":                    DXLRegister(65,1),
    "status_ret_level":       DXLRegister(68,1),
    "registered_instruction": DXLRegister(69,1),
    "hardware_err_status":    DXLRegister(70,1),
    "vel_I_gain":             DXLRegister(76,2),
    "vel_P_gain":             DXLRegister(78,2),
    "pos_D_gain":             DXLRegister(80,2),
    "pos_I_gain":             DXLRegister(82,2),
    "pos_P_gain":             DXLRegister(84,2),
    "feedfwd_2nd_gain":       DXLRegister(88,2),
    "feedfwd_1st_gain":       DXLRegister(90,2),
    "bus_watchdog":           DXLRegister(98,1),
    "goal_pwm":               DXLRegister(100,2),
    "goal_vel":               DXLRegister(104,4),
    "profile_accel":          DXLRegister(108,4),
    "profile_vel":            DXLRegister(112,4),
    "goal_pos":               DXLRegister(116,4),
    "realtime_tick":          DXLRegister(120,2),
    "moving":                 DXLRegister(122,1),
    "moving_status":          DXLRegister(123,1),
    "present_pwm":            DXLRegister(124,2),
    "present_load":           DXLRegister(126,2),
    "present_vel":            DXLRegister(128,4),
    "present_pos":            DXLRegister(132,4),
    "vel_trajectory":         DXLRegister(136,4),
    "pos_trajectory":         DXLRegister(140,4),
    "present_input_volt":     DXLRegister(144,2),
    "present_temp":           DXLRegister(146,1),
    "backup_ready":           DXLRegister(147,1),
}

class Dynamixel_Controller:
    ### constructor

    def __init__(self, dev_name: str, baud: int, dxl_ids: List[int]) -> None:
        
        self.port_hand = dxl.PortHandler(dev_name)

        if baud not in valid_bauds_list:
            raise ValueError("Invalid baud rate specified.\nPlease choose one of the following valid baud rates:\n9600\n57600\n115200\n1000000\n2000000\n3000000\n4000000\n4500000")
        
        # TODO check this?
        # if not isinstance(dxl_ids, List[int]):
        #     raise ValueError("Argument <dxl_ids> must be of type List[int]")
        self.ids_list = dxl_ids

        self.packet_hand = dxl.PacketHandler(PROTOCOL_VER)

        # Set port baudrate
        if self.port_hand.setBaudRate(baud):
            print(f"Dynamixel Controller baud set to {baud}")
        else:
            print("Failed to change Dynamixel Controller baud rate")

    ### convenience functions

    def set_goal_pos(self, dxl_id: int, pos: int) -> None:
        self.write_all("torque_enable", ENABLE)
        self.write_register(dxl_id, "goal_pos", pos)

    def get_curr_pos(self, dxl_id: int) -> int:
        return self.read_register(dxl_id, "present_pos")
    
    def set_op_mode_all(self, mode: int) -> None:
        self.write_all("op_mode", mode)

    def enable_torque_all(self) -> None:
        self.write_all("torque_enable", ENABLE)

    def disable_torque_all(self) -> None:
        self.write_all("torque_enable", DISABLE)

    def at_goal_pos(self, dxl_id: int) -> bool:
        # bit 0 of this register indicates the status (0=not at pos, 1=at pos)
        return bool(self.read_register(dxl_id, "moving_status") & 1)
    
    def reboot(self, dxl_id: int) -> None:
        if not self.port_hand.openPort():
            print("Failed to open Dynamixel Controller port; write operation not performed")
            return      
         
        print("1")
        self.packet_hand.reboot(self.port_hand, dxl_id)
        print("2")
        # self.packet_hand.txPacket(self.port_hand, dxl_id)
        self.enable_torque_all()

    ### basic low-level helpers

    def write_register(self, dxl_id: int, reg_name: str, val: int) -> None:
        if not self.port_hand.openPort():
            print("Failed to open Dynamixel Controller port; write operation not performed")
            return
        
        if dxl_id not in self.ids_list:
            print(f"No Dynamixel on the bus with id <{dxl_id}>; write operation not performed")
            return
        
        register = reg_dict.get(reg_name)
        if(register is None):
            print(f"<{reg_name}> is not a valid Dynamixel register")
            return

        dxl_comm_result, dxl_error = self.packet_hand.writeTxRx(self.port_hand, dxl_id, register.addr, register.len, val.to_bytes(register.len, 'little', signed=True))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_hand.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_hand.getRxPacketError(dxl_error))

        self.port_hand.closePort()
        return

    def read_register(self, dxl_id: int, reg_name: str) -> int:
        if not self.port_hand.openPort():
            print("Failed to open Dynamixel Controller port; read operation not performed")
            return
        
        if dxl_id not in self.ids_list:
            print(f"No Dynamixel on the bus with id <{dxl_id}>; read operation not performed")
            return

        register = reg_dict.get(reg_name)
        # register not found in table
        if(register is None):
            print(f"<{reg_name}> is not a valid Dynamixel register")
            return None
        
        # use the length of the register being read to determine how many bytes to read
        # this is done because the generalized 'readTxRx' function returns bytes in an
        # awkward list format that is likely slower to convert
        if register.len == 1:
            ret, dxl_comm_result, dxl_error = self.packet_hand.read1ByteTxRx(self.port_hand, dxl_id, register.addr)
        elif register.len == 2:
            ret, dxl_comm_result, dxl_error = self.packet_hand.read2ByteTxRx(self.port_hand, dxl_id, register.addr)
        elif register.len == 4:
            ret, dxl_comm_result, dxl_error = self.packet_hand.read4ByteTxRx(self.port_hand, dxl_id, register.addr)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_hand.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_hand.getRxPacketError(dxl_error))

        self.port_hand.closePort()

        # converting from returned value to signed integer
        bits = 8*register.len
        if (ret & (1 << (bits - 1)) != 0): # if sign bit is set e.g., 8bit: 128-255
            ret = ret - (1 << bits)        # compute negative value

        return ret # TODO unsure what will be returned on a comms failure or error...
    
    # TODO it may be faster to use groupSyncRead here
    def write_all(self, reg_name: str, val: int) -> None:
        for dxl_id in self.ids_list:
            self.write_register(dxl_id, reg_name, val)
