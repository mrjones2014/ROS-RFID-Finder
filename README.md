# ROS-RFID-Finder
In this tutorial, we'll write a collection of ROS-enabled scripts in Python 2.7 for locating RFID tags using a Turtlebot Kobuki base, the Turtlebot's onboard camera, and an RFID reader; we're using [this RFID reader kit from SparkFun.](https://www.sparkfun.com/products/13198)

## General Note on Syntax for Terminal Snippets
For commands to be run in the terminal, lines following the `$` character are commands, anything following a `#` character is a comment, and anything else is output returned by the terminal commands.

# Prerequisites and Dependencies
In order to follow along with this tutorial, you must first ensure you have the following dependencies installed and configured properly:

1. Python 2 >=2.7
2. Pip (Pip is installed automatically with Python 2 >=2.7.9)
3. ROS-Indigo installed on Ubuntu 14.04 [(tutorial)](http://wiki.ros.org/indigo/Installation/Ubuntu)
4. OpenCV 3.0.0 or newer with Python 2.7+ bindings [(tutorial)](http://www.pyimagesearch.com/2015/06/22/install-opencv-3-0-and-python-2-7-on-ubuntu/)

To check that you have the required Python packages installed, open up a new terminal window and run `python` to open the Python terminal. Then, at the Python terminal, type `import rospy`; if the package is installed properly, you should get no output. Then, still in the Python terminal, type `import cv2`; again, if installed properly, you should get no output. To ensure that you have the correct version of OpenCV, in the Python terminal, run `cv2.__version__`; you should get '3.0.0' (or newer). Your full terminal session should look like this:
```bash
$ python
>>> import rospy
>>> import cv2
>> cv2.__version__
'3.1.0'
>>>
```
Press `ctrl+D` to close the Python terminal. Now that your dependencies are all set up and configured, we can begin creating our package.

# 1. Set up your environment
You can choose any directory for your package, but it must be added to the `ROS_PACKAGE_PATH` environment variable so that other tools like `rosrun` and `rospack` can find your new package. To do this, run `gedit ~/.bashrc` in a terminal window. When Gedit opens, scroll to the bottom and add the following lines:
```
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:<your-package-directory>
```
replacing `<your-package-directory>` with the directory you've chosen for the package, **ending in a slash**. For this tutorial, the directory is `~/ROS-RFID-Finder/`. Then, save and close the file. This will add your package's directory to the `ROS_PACKAGE_PATH` environment variable each time you open a new terminal session. Restart your terminal to apply these changes.

# 2. Create a new package
Now, we can begin creating our package. We'll be using the `rosbuild` build system. You may be thinking "Build system? For *Python*?". Yes, we need a build system and a `CMakeLists.txt` file for a Python project in ROS; this is to ensure that certain files are auto-generated, and so that the project is recognized as a ROS package by other tools like `rosrun` and `rospack`. However, none of these files need to be edited manually, as the `rosbuild` system will auto-generate everything you need, if you use it correctly. The first command we'll use is `roscreate-pkg`. The syntax is `roscreate-pkg <package-name> [dependencies...]`. To create your package, open a terminal and run the following commands:
```bash
$ cd ~/ROS-RFID-Finder
~/ROS-RFID-Finder$ roscreate-pkg rfid_finder std_msgs sensor_msgs geometry_msgs rospy
Created package directory /home/mat/ROS-RFID-Finder/rfid_finder
Created python source directory /home/mat/ROS-RFID-Finder/rfid_finder/src
Created package file ~/ROS-RFID-Finder/rfid_finder/Makefile
Created package file ~/ROS-RFID-Finder/rfid_finder/manifest.xml
Created package file ~/ROS-RFID-Finder/rfid_finder/CMakeLists.txt
Created package file ~/ROS-RFID-Finder/rfid_finder/mainpage.dox

Please edit rfid_finder/manifest.xml and mainpage.dox to finish creating your package
~/ROS-RFID-Finder$ rospack profile
```
Then, to confirm that your package has been created and registered successfully, run `rospack find rfid_finder`, and it should return the full path to the package directory.

Since we'll be using Python, we need to be working in a subdirectory called `scripts`. In the terminal window, run the following commands:
```bash
$ roscd rfid_finder
~/ROS-RFID-Finder/rfid_finder$ mkdir scripts && cd scripts
~/ROS-RFID-Finder/rfid_finder/scripts$
```
# 3. Install Python libraries
For our scripts, we'll use a few libraries in addition to OpenCV; `numpy`, `chardet`, and `pyserial`. You can install these packages by running `pip install numpy`, `pip install chardet`, and `pip install pyserial`. If any of these fail to install, try re-running the `pip install` command as sudo (e.g. `sudo pip install` the package). Here's an explanation of what each of these packages are for:

1. numpy - A standardized Python math library; used heavily by OpenCV (images are represented as numpy matricies)
2. chardet - A library for automatically detecting the encoding scheme of an encoded string
3. pyserial - A library for reading/writing data over a serial connection

# 4. Setting the permissions of the serial device
In order for `pyserial` to be able to read data from the RFID reader over a serial connection, you need to set the permissions of the serial device. First, run the command `dmesg | grep tty` to see a list of all connected serial devices. Then, connect the RFID reader using the USB cable and run `dmesg | grep tty` again; the device that was not listed the first time is your RFID reader. This will tell you which serial port your device is connected to. For me, it was ttyUSB0:
```bash 
$ dmesg | grep tty
[    0.000000] console [tty0] enabled
[ 6290.333239] usb 1-3: FTDI USB Serial Device converter now attached to ttyUSB0
```
Once you've identified the serial port ID (e.g. ttyUSB0 in my case), you can change the permissions for the device using the `chmod` command. Since we won't need to write to the device (only read from it), we'll just add the "read" permission by running the command `sudo chmod +r /dev/ttyUSB0` (if your serial ID was different, replace `ttyUSB0` with your serial ID). Here's my entire terminal session for this part of the tutorial:
```bash
$ dmesg | grep tty
[    0.000000] console [tty0] enabled
$ dmesg | grep tty
[    0.000000] console [tty0] enabled
[ 6290.333239] usb 1-3: FTDI USB Serial Device converter now attached to ttyUSB0
$ sudo chmod +r /dev/ttyUSB0
[sudo] password for root:
$
```
# 5. ROS-ify the RFID reader
In order to fully utilize the RFID reader, we need to ROS-ify it (e.g. write a ROS publisher node to publish data read from the device). Create a new file in the `scripts` subdirectory of the package we created earlier called `rfid_reader_node.py` and open the file in your favorite text editor; I'm using Jetbrains PyCharm Professional Edition 2016.

First, let's get our required imports set up:
```python 
#!/usr/bin/env python
import codecs
import chardet
import serial
import rospy
from std_msgs.msg import String
import argparse
```

1. `codecs` is required to decode the string into a human-readable format.
2. `chardet` is required to automatically detect the encoding of the data read from the device.
3. `serial` is required to read from the serial device.
4. `rospy` is required to ROS-ify the script
5. `String` from `std_msgs.msg` is required because its the message type we'll use to publish the data.
6. `argparse` is required to parse command line arguments.

First, let's parse our command line arguments using `argparse`:
```python
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--baud', help="The baud rate of the serial device. Default: 9600.", required=False)
    parser.add_argument('serialID', help="The serial device to read from. Example: ttyUSB0")
    args = parser.parse_args()
    
    serial_device = args.serialID
    baud_rate = 9600
    if args.baud:
        baud_rate = args.baud
```

Next, we need to initialize the ROS node and create a ROS topic and publisher. We'll give our node the name "rfid_reader", and our topic the name "rfid_data":
```python
rospy.init_node("rfid_reader")
pub = rospy.Publisher("rfid_data", String)
```
Then, all that's left is to actually read and publish the data. Since this will be a standalone node, all we need is a loop that constantly checks for data coming over the serial port, and when there is, decodes and publishes the data.
```python
while True:
    ser = serial.Serial('/dev/' + serial_device, baud_rate)
    data = ser.readline()[1:]  # throw out first byte; it comes back garbled and unreadable
    encoding = chardet.detect(data).values()[1]  # the encoding scheme name as a string
    decoded = codecs.decode(data, encoding)
    pub.publish(decoded)
```

That's it! The full script should look like this:
```python
#!/usr/bin/env python
import codecs
import chardet
import serial
import rospy
from std_msgs.msg import String
import argparse


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
    pub = rospy.Publisher("rfid_data", String)

    while True:
        ser = serial.Serial('/dev/' + serial_device, baud_rate)
        data = ser.readline()[1:]  # throw out first byte; it comes back garbled and unreadable
        encoding = chardet.detect(data).values()[1]  # the encoding scheme name as a string
        decoded = codecs.decode(data, encoding)
        pub.publish(decoded)
```
Now we can build the package by running the following commands:
```bash
$ roscd rfid_finder
~/ROS-RFID-Finder/rfid_finder$ rosmake
```
This will generate all the required message and build files. Now, we need to make our Python script executable by running the following commands:
```bash
$ roscd rfid_finder
~/ROS-RFID-Finder/rfid_finder$ cd scripts
~/ROS-RFID-Finder/rfid_finder/scripts$ chmod +x rfid_reader_node.py
```
Now, you should be able to start your node (assuming `roscore` is running) by running the following command at a terminal:
```bash
$ rosrun rfid_finder rfid_reader_node.py ttyUSB0
```
