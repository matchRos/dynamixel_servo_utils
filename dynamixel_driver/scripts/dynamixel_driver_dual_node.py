#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import rospy
from std_msgs.msg import Float32, String
from dynamixel_sdk import PortHandler, PacketHandler


class MotorState:
    def __init__(self, dxl_id, sign):
        self.dxl_id = dxl_id
        self.sign = sign  # +1 or -1 for opposite direction
        self.last_present_current_ma = 0.0
        self.last_present_rpm = 0.0
        self.goal_rpm_limited_active = False
        self.goal_rpm_limited = 0.0


class DynamixelDualDriverNode:
    def __init__(self):
        # Parameters
        self.device_name = rospy.get_param("~device_name", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~baudrate", 57600)
        self.protocol_version = rospy.get_param("~protocol_version", 2.0)
        self.read_rate = rospy.get_param("~read_rate", 20.0)

        self.dxl_id_1 = rospy.get_param("~dxl_id_1", 1)
        self.dxl_id_2 = rospy.get_param("~dxl_id_2", 2)

        self.max_current_ma = rospy.get_param("~max_current_ma", 50.0)
        self.kp_rpm_to_current = rospy.get_param("~kp_rpm_to_current", 1.0)

        # XL330 registers
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

        self.is_shutting_down = False
        self.dxl_lock = threading.Lock()

        self.active_mode_name = "unknown"
        self.current_mode = (
            None  # shared assumption: both motors always kept in same mode
        )

        self.motor1 = MotorState(self.dxl_id_1, +1)
        self.motor2 = MotorState(self.dxl_id_2, -1)

        # SDK
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        # Publishers
        self.pub_present_current_1 = rospy.Publisher(
            "/motor1/present_current", Float32, queue_size=10
        )
        self.pub_present_rpm_1 = rospy.Publisher(
            "/motor1/present_rpm", Float32, queue_size=10
        )

        self.pub_present_current_2 = rospy.Publisher(
            "/motor2/present_current", Float32, queue_size=10
        )
        self.pub_present_rpm_2 = rospy.Publisher(
            "/motor2/present_rpm", Float32, queue_size=10
        )

        self.pub_active_mode = rospy.Publisher("/active_mode", String, queue_size=10)

        # Subscribers
        self.sub_goal_current = rospy.Subscriber(
            "/goal_current", Float32, self.goal_current_cb
        )
        self.sub_goal_rpm = rospy.Subscriber("/goal_rpm", Float32, self.goal_rpm_cb)
        self.sub_goal_rpm_limited = rospy.Subscriber(
            "/goal_rpm_limited", Float32, self.goal_rpm_limited_cb
        )
        self.sub_max_current = rospy.Subscriber(
            "/max_current", Float32, self.max_current_cb
        )

        self.init_dynamixels()

    def dxl_to_signed(self, val, bits):
        if val >= (1 << (bits - 1)):
            val -= 1 << bits
        return val

    def signed_to_dxl(self, val, bits):
        if val < 0:
            val += 1 << bits
        return val

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def check_result(self, dxl_comm_result, dxl_error, action_name):
        if dxl_comm_result != 0:
            rospy.logerr(
                "%s failed: %s",
                action_name,
                self.packet_handler.getTxRxResult(dxl_comm_result),
            )
            return False
        if dxl_error != 0:
            rospy.logerr(
                "%s DXL error: %s",
                action_name,
                self.packet_handler.getRxPacketError(dxl_error),
            )
            return False
        return True

    def mode_to_name(self, mode):
        if mode == self.mode_current:
            return "current"
        if mode == self.mode_velocity:
            return "velocity"
        return str(mode)

    def read1(self, dxl_id, addr, name):
        with self.dxl_lock:
            value, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
                self.port_handler, dxl_id, addr
            )
        if not self.check_result(dxl_comm_result, dxl_error, name):
            return None
        return value

    def read2(self, dxl_id, addr, name):
        with self.dxl_lock:
            value, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
                self.port_handler, dxl_id, addr
            )
        if not self.check_result(dxl_comm_result, dxl_error, name):
            return None
        return value

    def read4(self, dxl_id, addr, name):
        with self.dxl_lock:
            value, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                self.port_handler, dxl_id, addr
            )
        if not self.check_result(dxl_comm_result, dxl_error, name):
            return None
        return value

    def write1(self, dxl_id, addr, value, name):
        with self.dxl_lock:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, addr, value
            )
        return self.check_result(dxl_comm_result, dxl_error, name)

    def write2(self, dxl_id, addr, value, name):
        with self.dxl_lock:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler, dxl_id, addr, value
            )
        return self.check_result(dxl_comm_result, dxl_error, name)

    def write4(self, dxl_id, addr, value, name):
        with self.dxl_lock:
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id, addr, value
            )
        return self.check_result(dxl_comm_result, dxl_error, name)

    def init_dynamixels(self):
        with self.dxl_lock:
            if not self.port_handler.openPort():
                rospy.logfatal("Failed to open port: %s", self.device_name)
                raise RuntimeError("Failed to open port")

            rospy.loginfo("Opened port: %s", self.device_name)

            if not self.port_handler.setBaudRate(self.baudrate):
                rospy.logfatal("Failed to set baudrate: %d", self.baudrate)
                raise RuntimeError("Failed to set baudrate")

            rospy.loginfo("Set baudrate: %d", self.baudrate)

        mode1 = self.read1(
            self.motor1.dxl_id, self.addr_operating_mode, "Read operating mode motor1"
        )
        mode2 = self.read1(
            self.motor2.dxl_id, self.addr_operating_mode, "Read operating mode motor2"
        )

        if mode1 is None or mode2 is None:
            raise RuntimeError("Failed to read operating mode")

        if mode1 != mode2:
            rospy.logwarn(
                "Motors start in different modes: motor1=%d motor2=%d. Normalizing to motor1 mode.",
                mode1,
                mode2,
            )

        self.current_mode = mode1
        self.active_mode_name = self.mode_to_name(self.current_mode)

        if not self.write1(
            self.motor1.dxl_id,
            self.addr_torque_enable,
            self.torque_enable,
            "Enable torque motor1",
        ):
            raise RuntimeError("Failed to enable torque motor1")
        if not self.write1(
            self.motor2.dxl_id,
            self.addr_torque_enable,
            self.torque_enable,
            "Enable torque motor2",
        ):
            raise RuntimeError("Failed to enable torque motor2")

        rospy.loginfo("Torque enabled on both motors")

    def set_operating_mode_both(self, new_mode):
        if self.current_mode == new_mode:
            return True

        # torque off
        if not self.write1(
            self.motor1.dxl_id,
            self.addr_torque_enable,
            self.torque_disable,
            "Disable torque motor1 for mode switch",
        ):
            return False
        if not self.write1(
            self.motor2.dxl_id,
            self.addr_torque_enable,
            self.torque_disable,
            "Disable torque motor2 for mode switch",
        ):
            return False

        # write mode
        if not self.write1(
            self.motor1.dxl_id,
            self.addr_operating_mode,
            new_mode,
            "Write operating mode motor1",
        ):
            return False
        if not self.write1(
            self.motor2.dxl_id,
            self.addr_operating_mode,
            new_mode,
            "Write operating mode motor2",
        ):
            return False

        # torque on
        if not self.write1(
            self.motor1.dxl_id,
            self.addr_torque_enable,
            self.torque_enable,
            "Re-enable torque motor1",
        ):
            return False
        if not self.write1(
            self.motor2.dxl_id,
            self.addr_torque_enable,
            self.torque_enable,
            "Re-enable torque motor2",
        ):
            return False

        self.current_mode = new_mode
        self.active_mode_name = self.mode_to_name(new_mode)
        rospy.loginfo("Switched both motors to mode %s", self.active_mode_name)
        return True

    def write_goal_current_ma(self, motor, current_ma):
        current_ma_int = int(round(current_ma))
        current_raw = self.signed_to_dxl(current_ma_int, 16)
        return self.write2(
            motor.dxl_id,
            self.addr_goal_current,
            current_raw,
            f"Write goal current motor {motor.dxl_id}",
        )

    def write_goal_velocity_rpm(self, motor, rpm):
        velocity_raw_signed = int(round(rpm / 0.229))
        velocity_raw = self.signed_to_dxl(velocity_raw_signed, 32)
        return self.write4(
            motor.dxl_id,
            self.addr_goal_velocity,
            velocity_raw,
            f"Write goal velocity motor {motor.dxl_id}",
        )

    def disable_limited_mode(self):
        self.motor1.goal_rpm_limited_active = False
        self.motor2.goal_rpm_limited_active = False

    def goal_current_cb(self, msg):
        if self.is_shutting_down:
            return

        self.disable_limited_mode()

        if not self.set_operating_mode_both(self.mode_current):
            return

        base_current_ma = self.clamp(
            msg.data, -self.max_current_ma, self.max_current_ma
        )

        self.write_goal_current_ma(self.motor1, self.motor1.sign * base_current_ma)
        self.write_goal_current_ma(self.motor2, self.motor2.sign * base_current_ma)

    def goal_rpm_cb(self, msg):
        if self.is_shutting_down:
            return

        self.disable_limited_mode()

        if not self.set_operating_mode_both(self.mode_velocity):
            return

        base_rpm = msg.data

        self.write_goal_velocity_rpm(self.motor1, self.motor1.sign * base_rpm)
        self.write_goal_velocity_rpm(self.motor2, self.motor2.sign * base_rpm)

    def goal_rpm_limited_cb(self, msg):
        if self.is_shutting_down:
            return

        base_rpm = msg.data

        self.motor1.goal_rpm_limited = self.motor1.sign * base_rpm
        self.motor2.goal_rpm_limited = self.motor2.sign * base_rpm
        self.motor1.goal_rpm_limited_active = True
        self.motor2.goal_rpm_limited_active = True

        if not self.set_operating_mode_both(self.mode_current):
            return

        self.active_mode_name = "rpm_limited"

    def max_current_cb(self, msg):
        if self.is_shutting_down:
            return

        self.max_current_ma = abs(msg.data)
        rospy.loginfo("Updated max_current_ma: %.2f", self.max_current_ma)

        if self.motor1.goal_rpm_limited_active or self.motor2.goal_rpm_limited_active:
            self.apply_limited_rpm_control(self.motor1)
            self.apply_limited_rpm_control(self.motor2)

    def read_present_state(self, motor):
        present_current_raw = self.read2(
            motor.dxl_id,
            self.addr_present_current,
            f"Read present current motor {motor.dxl_id}",
        )
        if present_current_raw is not None:
            motor.last_present_current_ma = float(
                self.dxl_to_signed(present_current_raw, 16)
            )

        present_velocity_raw = self.read4(
            motor.dxl_id,
            self.addr_present_velocity,
            f"Read present velocity motor {motor.dxl_id}",
        )
        if present_velocity_raw is not None:
            present_velocity_signed = self.dxl_to_signed(present_velocity_raw, 32)
            motor.last_present_rpm = float(present_velocity_signed) * 0.229

    def apply_limited_rpm_control(self, motor):
        if not motor.goal_rpm_limited_active:
            return

        rpm_error = motor.goal_rpm_limited - motor.last_present_rpm
        commanded_current = self.kp_rpm_to_current * rpm_error
        commanded_current = self.clamp(
            commanded_current, -self.max_current_ma, self.max_current_ma
        )

        self.write_goal_current_ma(motor, commanded_current)

    def read_and_publish(self):
        if self.is_shutting_down:
            return

        self.read_present_state(self.motor1)
        self.read_present_state(self.motor2)

        if self.motor1.goal_rpm_limited_active or self.motor2.goal_rpm_limited_active:
            if self.current_mode != self.mode_current:
                if not self.set_operating_mode_both(self.mode_current):
                    return

            self.active_mode_name = "rpm_limited"
            self.apply_limited_rpm_control(self.motor1)
            self.apply_limited_rpm_control(self.motor2)
        else:
            self.active_mode_name = self.mode_to_name(self.current_mode)

        self.pub_present_current_1.publish(self.motor1.last_present_current_ma)
        self.pub_present_rpm_1.publish(self.motor1.last_present_rpm)

        self.pub_present_current_2.publish(self.motor2.last_present_current_ma)
        self.pub_present_rpm_2.publish(self.motor2.last_present_rpm)

        self.pub_active_mode.publish(self.active_mode_name)

    def shutdown(self):
        self.is_shutting_down = True
        rospy.loginfo("Shutting down Dynamixel dual driver node")

        for motor in [self.motor1, self.motor2]:
            try:
                self.write2(
                    motor.dxl_id,
                    self.addr_goal_current,
                    0,
                    f"Reset goal current motor {motor.dxl_id}",
                )
            except Exception as e:
                rospy.logwarn(
                    "Could not reset goal current motor %d: %s", motor.dxl_id, str(e)
                )

            try:
                self.write4(
                    motor.dxl_id,
                    self.addr_goal_velocity,
                    0,
                    f"Reset goal velocity motor {motor.dxl_id}",
                )
            except Exception as e:
                rospy.logwarn(
                    "Could not reset goal velocity motor %d: %s", motor.dxl_id, str(e)
                )

            try:
                self.write1(
                    motor.dxl_id,
                    self.addr_torque_enable,
                    self.torque_disable,
                    f"Disable torque motor {motor.dxl_id}",
                )
            except Exception as e:
                rospy.logwarn(
                    "Could not disable torque motor %d: %s", motor.dxl_id, str(e)
                )

        with self.dxl_lock:
            try:
                self.port_handler.closePort()
            except Exception as e:
                rospy.logwarn("Could not close port: %s", str(e))

        rospy.loginfo("Port closed")


if __name__ == "__main__":
    rospy.init_node("dynamixel_driver_dual_node")

    node = DynamixelDualDriverNode()
    rospy.on_shutdown(node.shutdown)

    rate = rospy.Rate(node.read_rate)
    while not rospy.is_shutdown():
        node.read_and_publish()
        rate.sleep()
