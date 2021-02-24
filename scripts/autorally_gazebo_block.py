#!/usr/bin/python2
import rospy
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, Imu
from autorally_msgs.msg import chassisCommand, chassisState, wheelSpeeds
from nav_msgs.msg import Odometry

import rospkg
import subprocess
import time
import roslaunch
import os

# ros node initialization
rospy.init_node('autorally_gazebo_block', anonymous=True)

# starting TurboVNC
print "Deleting Dispaly files"
subprocess.Popen("ls /tmp/.X11-unix/", shell=True)
subprocess.Popen("rm -rf /tmp/.X11-unix/X1", shell=True)
print "Starting Vnc:"
subprocess.Popen(
    "sh /usr/local/bin/start_desktop.sh", shell=True)
time.sleep(5)


subprocess.Popen("echo $DISPLAY", shell=True)

# locating autorally gazebo launch file
rospack = rospkg.RosPack()
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
launch_path = rospack.get_path('autorally_gazebo') + '/launch/autoRallyTrackGazeboSim.launch'

# starting the launch file
gazebo_p = subprocess.Popen('vglrun roslaunch {} &'.format(launch_path), shell=True)
print "starting autorally block launch file!"

# pub_leftcam = rospy.Publisher('/block/left_camera/image_raw', Image, queue_size=10)
pub_rightcam = rospy.Publisher('/block/right_camera/image_raw', Image, queue_size=10)
# pub_leftcaminfo = rospy.Publisher('/block/left_camera/camera_info', CameraInfo, queue_size=10)
# pub_rightcaminfo = rospy.Publisher('/block/right_camera/camera_info', CameraInfo, queue_size=10)
pub_chassisState = rospy.Publisher('/block/chassisState', chassisState, queue_size=10)
pub_wheelSpeeds = rospy.Publisher('/block/wheelSpeeds', wheelSpeeds, queue_size=10)
pub_GPS = rospy.Publisher('/block/gpsRoverStatus', NavSatFix, queue_size=10)
pub_IMU = rospy.Publisher('/block/imu/imu', Imu, queue_size=10)
# pub_gt = rospy.Publisher('/block/ground_truth/state', Odometry, queue_size=10)

pub_command = rospy.Publisher('/OCS/chassisCommand', chassisCommand, queue_size=10)


def commandCallback(data_command):
    # print "got command"
    pub_command.publish(data_command)

def rightcamCallback(data_rightcam):
    print "got right cam"
    pub_rightcam.publish(data_rightcam)

# def leftcamCallback(data_leftcam):
#     print "got left cam"
#     pub_leftcam.publish(data_leftcam)

# def leftcaminfoCallback(data_leftcaminfo):
#     print "got left cam info"
#     pub_leftcaminfo.publish(data_leftcaminfo)

# def rightcaminfoCallback(data_rightcaminfo):
#     print "got right cam info"
#     pub_rightcaminfo.publish(data_rightcaminfo)

def chassisStateCallback(data_chassisState):
    # print "got chassis state"
    pub_chassisState.publish(data_chassisState)

def wheelSpeedsCallback(data_wheelSpeeds):
    # print "got wheel speeds"
    pub_wheelSpeeds.publish(data_wheelSpeeds)

def IMUCallback(data_IMU):
    # print "got IMU"
    pub_IMU.publish(data_IMU)

def GPSCallback(data_GPS):
    # print "got GPS"
    pub_GPS.publish(data_GPS)

# def gtCallback(data_gt):
#     # print "got ground truth"
#     pub_gt.publish(data_gt)

rospy.Subscriber("/block/OCS/chassisCommand", chassisCommand, commandCallback)
rospy.Subscriber("/right_camera/image_raw", Image, rightcamCallback)
# rospy.Subscriber("/left_camera/image_raw", Image, leftcamCallback)
# rospy.Subscriber("/right_camera/camera_info", CameraInfo, rightcaminfoCallback)
# rospy.Subscriber("/left_camera/camera_info", CameraInfo, leftcaminfoCallback)
rospy.Subscriber("/chassisState", chassisState, chassisStateCallback)
rospy.Subscriber("/wheelSpeeds", wheelSpeeds, wheelSpeedsCallback)
rospy.Subscriber("/gpsRoverStatus", NavSatFix, GPSCallback)
rospy.Subscriber("/imu/imu", Imu, IMUCallback)
# rospy.Subscriber("/ground_truth/state", Odometry, gtCallback)

rospy.spin()