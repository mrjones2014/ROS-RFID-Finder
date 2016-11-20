#!/usr/bin/env python
import codecs
import chardet
import serial
import rospy
from std_msgs.msg import String
import argparse
import sys


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--baud', help="The baud rate of the serial device. Default: 9600.", required=False)
    parser.add_argument('serialID', help="The serial device to read from. Example: ttyUSB0")
    args = parser.parse_args()

    serial_device = args.serialID
    baud_rate = 9600
    if args.baud:
        baud_rate = args.baud

    rospy.init_node("rfid_reader")
    pub = rospy.Publisher("rfid_data", String, queue_size=10)

    ser = serial.Serial('/dev/' + serial_device, baud_rate)
    while True:
        try:
            data = ser.readline()
            encoding = chardet.detect(data).values()[1]  # the encoding scheme name as a string
            decoded = codecs.decode(data, encoding)
            print "Encoding: ", encoding, "\nData: ", decoded, "\n"
            pub.publish(decoded)
        except Exception, e:
            sys.exit(0)

