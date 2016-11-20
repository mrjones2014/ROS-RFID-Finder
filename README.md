# ROS-RFID-Finder
In this tutorial, we'll write a collection of ROS-enabled scripts in Python 2.7 for locating RFID tags using a Turtlebot Kobuki base, the Turtlebot's onboard camera, and an RFID reader; we're using [this RFID reader kit from SparkFun.](https://www.sparkfun.com/products/13198)

## General Note on Syntax for Terminal Snippets
For commands to be run in the terminal, lines following the `$` character are commands, anything following a `#` character is a comment, and anything else is output returned by the terminal commands.

# Prerequisites
In order to follow along with this tutorial, you must first ensure you have the following dependencies installed and configured properly:

1. ROS-Indigo installed on Ubuntu 14.04 [(tutorial)](http://wiki.ros.org/indigo/Installation/Ubuntu)
2. Python 2.7
3. OpenCV 3.0.0 or newer with Python 2.7+ bindings [(tutorial)](http://www.pyimagesearch.com/2015/06/22/install-opencv-3-0-and-python-2-7-on-ubuntu/)

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
