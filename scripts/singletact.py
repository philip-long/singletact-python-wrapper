#!/usr/bin/env python
import numpy
import serial
import sys
import glob
import singletact_library
import time
import rospy
from std_msgs.msg import Float32


def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def get_port_name():
    port_name = rospy.get_param('~port')
    if not port_name:
        rospy.logwarn("No port name give from roslaunch. \n Searching ports for sensor")
        list_of_serial_ports = serial_ports()  # Equivalent to serialPortName in Gui.cs
        # Assume there is only one serial port
        if len(list_of_serial_ports) > 1:
            rospy.logwarn("Taking the first serial port, need to add a selector here")
        time.sleep(1)
        try:
            rospy.loginfo("Communicating on %s", list_of_serial_ports[0])
            port_name=list_of_serial_ports[0]
        except IndexError as x:
            rospy.logerr("Can't detect any serial port:", x,
                         "\n If the sensor is plugged in you may not have permissions to read the port, \n try "
                         " 'sudo chmod 666 /dev/ttyACM0'   ")
            raise
    else:
        rospy.logwarn("Using param port name '%s' from roslaunch", port_name)
        time.sleep(1)
    return port_name


def run_singletact():
    rospy.init_node('single_tact', anonymous=True)
    port_name = get_port_name()

    # Todo create a new message time with more context and timestamp
    usb_device = singletact_library.USBDevice(port_name)  # Initialize function
    pub = rospy.Publisher('single_tact', Float32, queue_size=10)

    rate = rospy.Rate(200)  # 10hz
    rospy.loginfo("Firmware settings %s", usb_device.single_tact.firmwareVersion)
    rospy.loginfo("Is it calibrated %f", usb_device.single_tact.isCalibrated)
    usb_device.single_tact.PushSettingsToHardware()
    while not rospy.is_shutdown():
        frame = usb_device.single_tact.ReadSensorData()
        # print("Sensor reading ", frame.sensorData_[0])
        # print("Sensor time", frame.timeStamp_)
        delta = frame.timeStamp_ - usb_device.lastTimestamp
        usb_device.setTimestamp(frame.timeStamp_)
        sensor_reading = frame.sensorData_[0]
        pub.publish(sensor_reading)
        rate.sleep()


if __name__ == '__main__':
    try:
        run_singletact()
    except rospy.ROSInterruptException:
        pass
