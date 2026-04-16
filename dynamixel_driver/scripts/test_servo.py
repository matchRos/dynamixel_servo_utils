#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dynamixel_sdk import PortHandler, PacketHandler
import time

DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DXL_ID = 1

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
ADDR_PRESENT_CURRENT = 126

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

GOAL_CURRENT_MA = 40  # start very small


def dxl_to_signed(val, bits):
    if val >= (1 << (bits - 1)):
        val -= (1 << bits)
    return val


def check_result(packet_handler, dxl_comm_result, dxl_error, action_name):
    if dxl_comm_result != 0:
        print(f"{action_name} failed: {packet_handler.getTxRxResult(dxl_comm_result)}")
        return False
    if dxl_error != 0:
        print(f"{action_name} DXL error: {packet_handler.getRxPacketError(dxl_error)}")
        return False
    return True


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

    # Enable torque
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(
        port_handler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
    )
    if not check_result(packet_handler, dxl_comm_result, dxl_error, "Enable torque"):
        port_handler.closePort()
        return
    print("Torque enabled")

    # Write goal current
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(
        port_handler, DXL_ID, ADDR_GOAL_CURRENT, GOAL_CURRENT_MA
    )
    if not check_result(packet_handler, dxl_comm_result, dxl_error, "Write goal current"):
        port_handler.closePort()
        return
    print(f"Goal current written: {GOAL_CURRENT_MA} mA")

    time.sleep(1.0)

    # Read present current
    present_current_raw, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(
        port_handler, DXL_ID, ADDR_PRESENT_CURRENT
    )
    if not check_result(packet_handler, dxl_comm_result, dxl_error, "Read present current"):
        port_handler.closePort()
        return

    present_current_raw = dxl_to_signed(present_current_raw, 16)
    print(f"Present current: {present_current_raw} mA")

    # Set current back to zero
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(
        port_handler, DXL_ID, ADDR_GOAL_CURRENT, 0
    )
    if check_result(packet_handler, dxl_comm_result, dxl_error, "Reset goal current"):
        print("Goal current reset to 0 mA")

    # Disable torque again
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(
        port_handler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
    )
    if check_result(packet_handler, dxl_comm_result, dxl_error, "Disable torque"):
        print("Torque disabled")

    port_handler.closePort()


if __name__ == "__main__":
    main()