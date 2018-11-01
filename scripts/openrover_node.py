#!/usr/bin/env python
import serial
from serial.threaded import ReaderThread
import rospy
import geometry_msgs.msg
from openrover import OpenRover, OpenRoverProtocol
import diagnostic_updater
from rospy.rostime import wallsleep, Duration
import std_msgs.msg
from nav_msgs.msg import Odometry
import rr_openrover_basic.msg


class OpenRoverNode:
    rover = None
    timers = None

    def __init__(self):
        self.pub_fast = rospy.Publisher('/rr_openrover_basic/raw_fast_rate_data', rr_openrover_basic.msg.RawRrOpenroverBasicFastRateData, queue_size=1)
        self.pub_medium = rospy.Publisher('/rr_openrover_basic/raw_med_rate_data', rr_openrover_basic.msg.RawRrOpenroverBasicMedRateData, queue_size=1)
        self.pub_slow = rospy.Publisher('/rr_openrover_basic/raw_slow_rate_data', rr_openrover_basic.msg.RawRrOpenroverBasicSlowRateData, queue_size=1)
        self.pub_battery_a = rospy.Publisher('/rr_openrover_basic/batterystatus_a', rr_openrover_basic.msg.SmartBatteryStatus, queue_size=1)
        self.pub_battery_b = rospy.Publisher('/rr_openrover_basic/batterystatus_b', rr_openrover_basic.msg.SmartBatteryStatus, queue_size=1)

        self.odom_enc_pub = rospy.Publisher("odom_encoder", Odometry, queue_size=1)
        self.is_charging_pub = rospy.Publisher("charging", std_msgs.msg.Bool, queue_size=1)

        self.motor_speeds_pub = rospy.Publisher("motor_speeds_commanded", std_msgs.msg.std_msgs.msg.Int32MultiArray, queue_size=1)
        self.vel_calc_pub = rospy.Publisher("vel_calc_pub", std_msgs.msg.Float32MultiArray, queue_size=1)
        self.sub_velocity = rospy.Subscriber('/cmd_vel/managed', geometry_msgs.msg.TwistStamped, self.cmd_vel_cb, queue_size=1)

        self.rover = OpenRover()

        self.updater = diagnostic_updater.Updater()

        self.timers = []

    def start(self):
        self.timers.extend([
            rospy.Timer(Duration.from_sec(1), self.publish_fast),
            rospy.Timer(Duration.from_sec(3), self.publish_medium),
            rospy.Timer(Duration.from_sec(20), self.publish_slow),
        ])
        while not rospy.is_shutdown():
            rospy.loginfo('opening OpenRover device')

            try:
                device_name = '/dev/rover'
                rospy.loginfo('Connecting to %s', device_name)
                self.rover.open(device_name)

                rospy.loginfo('Connected to %s', device_name)
                while True:
                    self.rover.set_motor_speeds(0, 0, 0)
                    for i in range(0, 70, 2):
                        self.rover.request_data(i)

                    result = self.rover.get_data(40)
                    for i in range(10):
                        result = self.rover.get_data_synchronous(40)
                        assert result == 40621

                    self.rover.set_motor_speeds(0, 0, 0)
                    for i in range(5):
                        self.rover.set_fan_speed(200)
                        wallsleep(1)  # listen to the fan for a few seconds
                    self.rover.set_fan_speed(0)

            except serial.SerialException as e:
                rospy.logerr('Rover encountered an unhandled communication exception %s', e)
                rospy.rostime.wallsleep(10)
            except Exception as e:
                rospy.logerr('Rover encountered an unhandled exception %s', e)
                raise
            finally:
                self.rover.close()

    def cmd_vel_cb(self, twist_stamped):
        """

        :type twist_stamped: geometry_msgs.msg.TwistStamped
        """
        pass

    def publish_fast(self, event):
        self.pub_fast.publish(
            left_motor=self.rover.get_data(28),
            right_motor=self.rover.get_data(30),
            flipper_motor=self.rover.get_data(32),
        )

    def publish_medium(self, event):
        self.pub_medium.publish(
            reg_pwr_total_current=self.rover.get_data(0),
            reg_motor_fb_rpm_left=self.rover.get_data(2),
            reg_motor_fb_rpm_right=self.rover.get_data(4),
            reg_flipper_fb_position_pot1=self.rover.get_data(6),
            reg_flipper_fb_position_pot2=self.rover.get_data(8),
            reg_motor_fb_current_left=self.rover.get_data(10),
            reg_motor_fb_current_right=self.rover.get_data(12),
            reg_motor_charger_state=self.rover.get_data(38),
            reg_power_a_current=self.rover.get_data(42),
            reg_power_b_current=self.rover.get_data(44),
            reg_motor_flipper_angle=self.rover.get_data(46),
            battery_current_a=self.rover.get_data(68),
            battery_current_b=self.rover.get_data(70),
        )

    def publish_slow(self, event):
        self.pub_slow.publish(
            reg_motor_fault_flag_left=self.rover.get_data(18),
            reg_motor_temp_left=self.rover.get_data(20),
            reg_motor_temp_right=self.rover.get_data(22),
            reg_power_bat_voltage_a=self.rover.get_data(24),
            reg_power_bat_voltage_b=self.rover.get_data(26),
            reg_robot_rel_soc_a=self.rover.get_data(34),
            reg_robot_rel_soc_b=self.rover.get_data(36),
            battery_mode_a=self.rover.get_data(56),
            battery_mode_b=self.rover.get_data(58),
            battery_temp_a=self.rover.get_data(60),
            battery_temp_b=self.rover.get_data(62),
            battery_voltage_a=self.rover.get_data(64),
            battery_voltage_b=self.rover.get_data(66),
            buildno=self.rover.get_data(40),
        )


if __name__ == '__main__':
    rospy.loginfo("Starting node")
    rospy.init_node('openrover')

    while not rospy.is_shutdown():
        rospy.loginfo('opening OpenRover device')
        node = OpenRoverNode()
        node.start()

    rospy.spin()


class rover_diagnostic():

    def __init__(self):
        rospy.Subscriber("/raw_slow_rate_data", rr_openrover_basic.msg.RawRrOpenroverBasicSlowRateData, self.slow_data_cb)
        rospy.Subscriber("/rr_openrover_basic/raw_med_rate_data", rr_openrover_basic.msg.RawRrOpenroverBasicMedRateData, self.med_data_cb)
        rospy.Subscriber("/rr_openrover_basic/battery_status_a", rr_openrover_basic.msg.SmartBatteryStatus, self.battery_status_a_cb)
        rospy.Subscriber("/rr_openrover_basic/battery_status_b", rr_openrover_basic.msg.SmartBatteryStatus, self.battery_status_b_cb)
        rospy.Subscriber('/rr_openrover_basic/charging', std_msgs.msg.Bool, self.openrover_charging_cb)
        self.pub = rospy.Publisher('/inorbit/custom_data/0', std_msgs.msg.String, queue_size=5)

    def slow_data_cb(self, data):
        warn_msg = "Battery Levels [" + str(data.reg_robot_rel_soc_a) + ", " + str(
            data.reg_robot_rel_soc_b) + "]    Motor temps [" + str(data.reg_motor_temp_left) + ", " + str(
            data.reg_motor_temp_right) + "]"
        #        rospy.logwarn_throttle(60,warn_msg)

        self.pub.publish("Battery Cell 1 SOC=" + str(data.reg_robot_rel_soc_a))
        self.pub.publish("Battery Cell 2 SOC=" + str(data.reg_robot_rel_soc_b))
        self.pub.publish("Left Motor Temp=" + str(data.reg_motor_temp_left))
        self.pub.publish("Right Motor Temp=" + str(data.reg_motor_temp_right))
        self.pub.publish("Battery Mode A=" + str(data.battery_mode_a))
        self.pub.publish("Battery Mode B=" + str(data.battery_mode_b))
        self.pub.publish("Battery Temp A=" + str(data.battery_temp_a))
        self.pub.publish("Power Bat Voltage A=" + str(data.reg_power_bat_voltage_a))
        self.pub.publish("Power Bat Voltage B=" + str(data.reg_power_bat_voltage_b))
        self.pub.publish("Battery Voltage A=" + str(data.battery_voltage_a))
        self.pub.publish("Battery Voltage B=" + str(data.battery_voltage_b))

    def med_data_cb(self, data):
        self.pub.publish("Motor Feedback Current Left=" + str(data.reg_motor_fb_current_left))
        self.pub.publish("Motor Feedback Current Right=" + str(data.reg_motor_fb_current_right))
        self.pub.publish("Power A Current=" + str(data.reg_power_a_current))
        self.pub.publish("Power B Current=" + str(data.reg_power_b_current))
        self.pub.publish("Battery Current A=" + str(data.battery_current_a))
        self.pub.publish("Battery Current B=" + str(data.battery_current_b))

    def battery_status_a_cb(self, msg):
        for k in type(msg).__slots__:
            if k != 'header':
                self.pub.publish("Battery Status A." + k + '=' + str(int(getattr(msg, k))))

    def battery_status_b_cb(self, msg):
        for k in type(msg).__slots__:
            if k != 'header':
                self.pub.publish("Battery Status B." + k + '=' + str(int(getattr(msg, k))))

    def openrover_charging_cb(self, msg):
        self.pub.publish("Open Rover Charging=" + str(msg.data))
