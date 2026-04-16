#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
from dynamixel_sdk import PortHandler, PacketHandler


class DynamixelCurrentNode:
    def __init__(self):
        self.device_name = rospy.get_param("~device_name", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~baudrate", 57600)
        self.protocol_version = rospy.get_param("~protocol_version", 2.0)
        self.dxl_id = rospy.get_param("~dxl_id", 1)
        self.read_rate = rospy.get_param("~read_rate", 20.0)

        self.addr_torque_enable = 64
        self.addr_goal_current = 102
        self.addr_present_current = 126
        self.addr_present_velocity = 128

        self.torque_enable = 1
        self.torque_disable = 0

        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        self.pub_present_current = rospy.Publisher("/present_current", Float32, queue_size=10)
        self.pub_present_rpm = rospy.Publisher("/present_rpm", Float32, queue_size=10)
        self.sub_goal_current = rospy.Subscriber("/goal_current", Float32, self.goal_current_cb)

        self.init_dynamixel()

    def dxl_to_signed(self, val, bits):
        if val >= (1 << (bits - 1)):
            val -= (1 << bits)
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

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_torque_enable, self.torque_enable
        )
        if not self.check_result(dxl_comm_result, dxl_error, "Enable torque"):
            raise RuntimeError("Failed to enable torque")

        rospy.loginfo("Torque enabled")

    def goal_current_cb(self, msg):
        goal_current_ma = int(msg.data)

        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_goal_current, goal_current_ma
        )
        self.check_result(dxl_comm_result, dxl_error, "Write goal current")

    def read_and_publish(self):
        present_current_raw, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_present_current
        )
        if self.check_result(dxl_comm_result, dxl_error, "Read present current"):
            present_current_raw = self.dxl_to_signed(present_current_raw, 16)
            self.pub_present_current.publish(float(present_current_raw))

        present_velocity_raw, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_present_velocity
        )
        if self.check_result(dxl_comm_result, dxl_error, "Read present velocity"):
            present_velocity_raw = self.dxl_to_signed(present_velocity_raw, 32)
            present_velocity_rpm = float(present_velocity_raw) * 0.229
            self.pub_present_rpm.publish(present_velocity_rpm)

    def shutdown(self):
        rospy.loginfo("Shutting down Dynamixel node")

        self.packet_handler.write2ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_goal_current, 0
        )

        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, self.addr_torque_enable, self.torque_disable
        )

        self.port_handler.closePort()
        rospy.loginfo("Torque disabled and port closed")


if __name__ == "__main__":
    rospy.init_node("dynamixel_current_node")

    node = DynamixelCurrentNode()
    rospy.on_shutdown(node.shutdown)

    rate = rospy.Rate(node.read_rate)
    while not rospy.is_shutdown():
        node.read_and_publish()
        rate.sleep()