#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dynamixel_sdk import PortHandler, PacketHandler

DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DXL_ID = 1

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


def dxl_to_signed(val, bits):
    if val >= (1 << (bits - 1)):
        val -= (1 << bits)
    return val


def main():
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print(f"Failed to open port: {DEVICENAME}")
        return
    print(f"Opened port: {DEVICENAME}")

    if not port_handler.setBaudRate(BAUDRATE):
        print(f"Failed to set baudrate: {BAUDRATE}")
        port_handler.closePort()
        return
    print(f"Set baudrate: {BAUDRATE}")

    # Read operating mode
    operating_mode, dxl_comm_result, dxl_error = packet_handler.read1ByteTxRx(
        port_handler, DXL_ID, ADDR_OPERATING_MODE
    )
    if dxl_comm_result != 0:
        print("Read Operating Mode failed:", packet_handler.getTxRxResult(dxl_comm_result))
        return
    if dxl_error != 0:
        print("DXL error:", packet_handler.getRxPacketError(dxl_error))
        return
    print(f"Operating mode raw: {operating_mode}")

    # Read torque enable
    torque_enable, dxl_comm_result, dxl_error = packet_handler.read1ByteTxRx(
        port_handler, DXL_ID, ADDR_TORQUE_ENABLE
    )
    if dxl_comm_result != 0:
        print("Read Torque Enable failed:", packet_handler.getTxRxResult(dxl_comm_result))
        return
    if dxl_error != 0:
        print("DXL error:", packet_handler.getRxPacketError(dxl_error))
        return
    print(f"Torque enable: {torque_enable}")

    # Read present current
    present_current_raw, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(
        port_handler, DXL_ID, ADDR_PRESENT_CURRENT
    )
    if dxl_comm_result != 0:
        print("Read Present Current failed:", packet_handler.getTxRxResult(dxl_comm_result))
        return
    if dxl_error != 0:
        print("DXL error:", packet_handler.getRxPacketError(dxl_error))
        return
    present_current_raw = dxl_to_signed(present_current_raw, 16)
    print(f"Present current raw: {present_current_raw}   ({present_current_raw} mA)")

    # Read present velocity
    present_velocity_raw, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(
        port_handler, DXL_ID, ADDR_PRESENT_VELOCITY
    )
    if dxl_comm_result != 0:
        print("Read Present Velocity failed:", packet_handler.getTxRxResult(dxl_comm_result))
        return
    if dxl_error != 0:
        print("DXL error:", packet_handler.getRxPacketError(dxl_error))
        return
    present_velocity_raw = dxl_to_signed(present_velocity_raw, 32)
    present_velocity_rpm = present_velocity_raw * 0.229
    print(f"Present velocity raw: {present_velocity_raw}   ({present_velocity_rpm:.3f} rpm)")

    port_handler.closePort()


if __name__ == "__main__":
    main()