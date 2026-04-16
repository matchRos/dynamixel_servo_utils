#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32, String
from dynamixel_sdk import PortHandler, PacketHandler


class DynamixelDriverNode:
    def __init__(self):
        self.device_name = rospy.get_param("~device_name", "/dev/ttyUSB1")
        self.baudrate = rospy.get_param("~baudrate", 57600)
        self.protocol_version = rospy.get_param("~protocol_version", 2.0)
        self.dxl_id = rospy.get_param("~dxl_id", 1)
        self.read_rate = rospy.get_param("~read_rate", 20.0)

        self.addr_operating_mode = 11
        self.addr_torque_enable = 64
        self.addr_goal_current = 102
        self.addr_goal_velocity = 104
        self.addr_present_current = 126
        self.addr_present_velocity = 128

        self.mode_current = 0
        self.mode_velocity = 1

        self.torque_enable = 1
        self.torque_disable = 0

        self.current_mode = None

        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        self.pub_present_current = rospy.Publisher("/present_current", Float32, queue_size=10)
        self.pub_present_rpm = rospy.Publisher("/present_rpm", Float32, queue_size=10)
        self.pub_active_mode = rospy.Publisher("/active_mode", String, queue_size=10)

        self.sub_goal_current = rospy.Subscriber("/goal_current", Float32, self.goal_current_cb)
        self.sub_goal_rpm = rospy.Subscriber("/goal_rpm", Float32, self.goal_rpm_cb)

        self.init_dynamixel()

    def dxl_to_signed(self, val, bits):
        if val >= (1 << (bits - 1)):
            val -= (1 << bits)
        return val

    def signed_to_dxl(self, val, bits):
        if val < 0:
            val += (1 << bits)
        return val

    def check_result(self, dxl_comm_result, dxl_error, action_name):
        if dxl_comm_result != 0:
            rospy.logerr("%s failed: %s", action_name, self.packet_handler.getTxRxResult(dxl_comm_result))
            return False
        if dxl_error != 0:
            rospy.logerr("%s DXL error: %s", action_name, self.packet_handler.getRxPacketError(dxl_error))
            return False
        return True

    def init_dynamixel(self):
        if not self.port_handler.openPort():
            rospy.logfatal("Failed to open port: %s", self.device_name)
            raise RuntimeError("Failed to open port")

        rospy.loginfo("Opened port: %s", self.device_name)

        if not self.port_handler.setBaudRate(self.baudrate):
            rospy.logfatal("Failed to set baudrate: %d", self.baudrate)
            raise RuntimeError("Failed to set baudrate")

        rospy.loginfo("Set baudrate: %d", self.baudrate)

        # Read current mode from servo
        mode, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_operating_mode
        )
        if not self.check_result(dxl_comm_result, dxl_error, "Read operating mode"):
            raise RuntimeError("Failed to read operating mode")

        self.current_mode = mode
        rospy.loginfo("Initial operating mode: %d", self.current_mode)

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_torque_enable, self.torque_enable
        )
        if not self.check_result(dxl_comm_result, dxl_error, "Enable torque"):
            raise RuntimeError("Failed to enable torque")

        rospy.loginfo("Torque enabled")

    def set_operating_mode(self, new_mode):
        if self.current_mode == new_mode:
            return True

        # Torque OFF before changing mode
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_torque_enable, self.torque_disable
        )
        if not self.check_result(dxl_comm_result, dxl_error, "Disable torque for mode switch"):
            return False

        # Write new mode
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_operating_mode, new_mode
        )
        if not self.check_result(dxl_comm_result, dxl_error, "Write operating mode"):
            return False

        # Torque ON again
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_torque_enable, self.torque_enable
        )
        if not self.check_result(dxl_comm_result, dxl_error, "Re-enable torque after mode switch"):
            return False

        self.current_mode = new_mode
        rospy.loginfo("Switched operating mode to %d", new_mode)
        return True

    def goal_current_cb(self, msg):
        if not self.set_operating_mode(self.mode_current):
            return

        goal_current_ma = int(round(msg.data))
        goal_current_raw = self.signed_to_dxl(goal_current_ma, 16)

        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_goal_current, goal_current_raw
        )
        self.check_result(dxl_comm_result, dxl_error, "Write goal current")

    def goal_rpm_cb(self, msg):
        if not self.set_operating_mode(self.mode_velocity):
            return

        goal_rpm = msg.data
        goal_velocity_raw_signed = int(round(goal_rpm / 0.229))
        goal_velocity_raw = self.signed_to_dxl(goal_velocity_raw_signed, 32)

        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_goal_velocity, goal_velocity_raw
        )
        self.check_result(dxl_comm_result, dxl_error, "Write goal velocity")

    def read_and_publish(self):
        present_current_raw, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_present_current
        )
        if self.check_result(dxl_comm_result, dxl_error, "Read present current"):
            present_current = float(self.dxl_to_signed(present_current_raw, 16))
            self.pub_present_current.publish(present_current)

        present_velocity_raw, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_present_velocity
        )
        if self.check_result(dxl_comm_result, dxl_error, "Read present velocity"):
            present_velocity_signed = self.dxl_to_signed(present_velocity_raw, 32)
            present_rpm = float(present_velocity_signed) * 0.229
            self.pub_present_rpm.publish(present_rpm)

        if self.current_mode == self.mode_current:
            self.pub_active_mode.publish("current")
        elif self.current_mode == self.mode_velocity:
            self.pub_active_mode.publish("velocity")
        else:
            self.pub_active_mode.publish(str(self.current_mode))

    def shutdown(self):
        rospy.loginfo("Shutting down Dynamixel driver node")

        # Set outputs to zero
        self.packet_handler.write2ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_goal_current, 0
        )
        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_goal_velocity, 0
        )

        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_torque_enable, self.torque_disable
        )
        self.port_handler.closePort()
        rospy.loginfo("Torque disabled and port closed")


if __name__ == "__main__":
    rospy.init_node("dynamixel_driver_node")

    node = DynamixelDriverNode()
    rospy.on_shutdown(node.shutdown)

    rate = rospy.Rate(node.read_rate)
    while not rospy.is_shutdown():
        node.read_and_publish()
        rate.sleep()