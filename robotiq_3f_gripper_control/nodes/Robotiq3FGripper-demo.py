#!/usr/bin/env python


# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
demo 3f robotiq gripper grips a cilinder bar, which is activated by fiber spot image intensity change,
i.e., open or close based on spot intensity change
"""

from __future__ import print_function

import roslib;

roslib.load_manifest('robotiq_3f_gripper_control')
import rospy
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

import rospy
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from std_msgs.msg import Float64

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as image_msg
import cv2 as cv

from numpy import empty

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
import math

from visulization.msg import IntList
import sys
import os



class gripperdemo:

  

  def __init__(self):
    
    self.tolerance = 10000 #image pixel difference for the detected cirlce
    np.set_printoptions(suppress=True)
    print("program starts")
    file = '2020-08-10-10_56_19 detected circle stats.csv'
    filename = '/home/ubuntu20/catkin_ws/src/PerceptionVisulization/logs/' + file
    if not os.path.exists(filename):
      print("Program aborted! initialization file: %s does NOT exists!" % filename)
      sys.exit(0)
    data_load = np.loadtxt(filename,delimiter=",", skiprows=1)

    self.historypixels = data_load[:,5]
    print("file name is ", file)
    print("is the initialization file correct? [Y/N]")
    if (input()=='Y'or'y'):

      print("data read from the initialization file is:")
      print(data_load)

      # rospy.init_node('realtimeprocess_image_subscriber', anonymous=True)
      # self.pub = rospy.Publisher('fiber_index', IntList,queue_size=10)
      self.pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput)

      self.command = Robotiq3FGripperRobotOutput();

      self.command = self.genCommand('r', self.command)

      self.pub.publish(self.command)
      rospy.sleep(0.1)

      input("press enter to continue...")
      # command = Robotiq3FGripperRobotOutput();
      self.command = self.genCommand('a', self.command)
      self.pub.publish(self.command)
      rospy.sleep(0.1)

      input("press enter to continue...")
      # command = Robotiq3FGripperRobotOutput();
      self.command = self.genCommand('c', self.command)
      self.pub.publish(self.command)
      rospy.sleep(0.1)


      input("press enter to continue...")
      # command = Robotiq3FGripperRobotOutput();
      self.command = self.genCommand('o', self.command)
      self.pub.publish(self.command)
      rospy.sleep(0.1)


      self.gripperstatus = 0 # rPRA in open status is 0

      print("gripper activated!!!")

      # rospy.Subscriber("image_raw", image_msg, callback, data_load, pub)
      self.sub = rospy.Subscriber("image_raw", image_msg, self.callback, data_load)

    else:
      print("Program aborted! Please update filename in image_realtime_process_from_camera.py file")

  def callback(self, data,args):
    stats_load = args
    # pub = args[1]
    bridge = CvBridge()
    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough') # output cv:mat
    except CvBridgeError as e:
      print(e)
    
    ####calculate the most pixelvalue changes
    maxchange, index = self.calculatepixelvalue(stats_load, cv_image)

    if abs(maxchange) > self.tolerance:
      # print("gripper is reacting...")
      self.gripperaction()
      rospy.sleep(1)
    

    # genCommand
  def calculatepixelvalue(self, stats_load, img):
    row_num = stats_load.shape[0]
    stats_load = np.array(stats_load)
    diff = []
    for i, stat in enumerate(stats_load):

      centerx = stat[3]
      centery = stat[2]
      radius = stat[4]
      x0 = int(centerx-radius)   ##opencv image has its x as column, y as row, orign at left top
      y0 = int(centery-radius)   ##here we use x0 as row in array, y0 as column in array

      diameter = int(2*radius)
      pixelvalue = 0
      count = 0 #number of cilcle pixel
      for x in range(x0,x0+diameter+1):
        for y in range(y0,y0+diameter+1):
          if math.sqrt((x-centerx)**2 + (y-centery)**2)<=radius:
            pixelvalue = pixelvalue+img[x,y]
      diff_current = self.historypixels[i]-pixelvalue
      diff.append(diff_current)
      # update history pixel value
      self.historypixels[i] = pixelvalue

    max_index = diff.index(max(diff))  #return fiber index
    max_value = max(diff)
    print("max diff:", max_value)

    # print("fibers responsed are: %d and %d, intensity changes are: %d and %d"\
    #       %(max1_index+1, max2_index+1, max1_value, max2_value))
    return [max_value, max_index]

  def gripperaction(self):
    # command = Robotiq3FGripperRobotOutput();
    self.command.rFRA = 10  #grip force [0-255]
    self.command.rSPA = 255 #speed [0-255]
    self.command.rMOD = 0 #normal mode
    # get gripper status
    if self.gripperstatus == 0: #fully open
      self.command.rPRA = 150  #position of the gripper [0-255], 0 for fully open, 255 for fully closed
      self.gripperstatus = 1
    else: #grasping object
      self.command.rPRA = 0
      self.gripperstatus = 0

    self.pub.publish(self.command)

    rospy.sleep(0.1) 

  def genCommand(self, char, command):
      #Update the command according to the character entered by the user.

    if char == 'a':
      command = Robotiq3FGripperRobotOutput();
      command.rACT = 1
      command.rGTO = 1
      command.rSPA = 255
      command.rFRA = 150

    if char == 'r':
      command = Robotiq3FGripperRobotOutput();
      command.rACT = 0

    if char == 'c':
      command.rPRA = 255

    if char == 'o':
      command.rPRA = 0

    if char == 'b':
      command.rMOD = 0

    if char == 'p':
      command.rMOD = 1

    if char == 'w':
      command.rMOD = 2

    if char == 's':
      command.rMOD = 3

    # If the command entered is a int, assign this value to rPRA
    try:
      command.rPRA = int(char)
      if command.rPRA > 255:
        command.rPRA = 255
      if command.rPRA < 0:
        command.rPRA = 0
    except ValueError:
      pass

    if char == 'f':
        command.rSPA += 25
        if command.rSPA > 255:
            command.rSPA = 255

    if char == 'l':
      command.rSPA -= 25
      if command.rSPA < 0:
        command.rSPA = 0

    if char == 'i':
      command.rFRA += 25
      if command.rFRA > 255:
        command.rFRA = 255

    if char == 'd':
      command.rFRA -= 25
      if command.rFRA < 0:
        command.rFRA = 0

    return command


def askForCommand(command):
    """Ask the user for a command to send to the gripper."""

    currentCommand = 'Simple 3F gripper Controller\n-----\nCurrent command:'
    currentCommand += ' rACT = ' + str(command.rACT)
    currentCommand += ', rMOD = ' + str(command.rMOD)
    currentCommand += ', rGTO = ' + str(command.rGTO)
    currentCommand += ', rATR = ' + str(command.rATR)
    ##    currentCommand += ', rGLV = ' + str(command.rGLV)
    ##    currentCommand += ', rICF = ' + str(command.rICF)
    ##    currentCommand += ', rICS = ' + str(command.rICS)
    currentCommand += ', rPRA = ' + str(command.rPRA)
    currentCommand += ', rSPA = ' + str(command.rSPA)
    currentCommand += ', rFRA = ' + str(command.rFRA)

    # We only show the simple control mode
    ##    currentCommand += ', rPRB = ' + str(command.rPRB)
    ##    currentCommand += ', rSPB = ' + str(command.rSPB)
    ##    currentCommand += ', rFRB = ' + str(command.rFRB)
    ##    currentCommand += ', rPRC = ' + str(command.rPRC)
    ##    currentCommand += ', rSPC = ' + str(command.rSPC)
    ##    currentCommand += ', rFRC = ' + str(command.rFRC)
    ##    currentCommand += ', rPRS = ' + str(command.rPRS)
    ##    currentCommand += ', rSPS = ' + str(command.rSPS)
    ##    currentCommand += ', rFRS = ' + str(command.rFRS)

    print(currentCommand)

    strAskForCommand = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += 'b: Basic mode\n'
    strAskForCommand += 'p: Pinch mode\n'
    strAskForCommand += 'w: Wide mode\n'
    strAskForCommand += 's: Scissor mode\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'

    strAskForCommand += '-->'

    #return raw_input(strAskForCommand)
    return input(strAskForCommand)



def publisher():
    """Main loop which requests new commands and publish them on the Robotiq3FGripperRobotOutput topic."""

    rospy.init_node('Robotiq3FGripperSimpleController')

    pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput)

    command = Robotiq3FGripperRobotOutput();

    while not rospy.is_shutdown():
        command = genCommand(askForCommand(command), command)

        pub.publish(command)

        rospy.sleep(0.1)


def main(args):
  rospy.init_node('Robotiq3FGripperDemo', anonymous=True)
  gp = gripperdemo()
  # rospy.init_node('image_converter', anonymous=True)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

