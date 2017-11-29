#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import picamera
from time import sleep
import base64
import random
import string
import math
from mqtt_video.msg import Image


def randomword(length):
	return ''.join(random.choice(string.lowercase) for i in range(length))
def take_picture():
	image_path = '/home/pi/Desktop/images/test_image.png'
	camera = picamera.PiCamera()
	try:
		#camera.start_preview()
		#sleep(1)
		camera.capture(image_path, format='png', resize=(400,400), quality=25)
		#camera.stop_preview()
		pass
	finally:
		camera.close()
def convertImageToBase64():
	image_path = '/home/pi/Desktop/images/test_image.png'
	with open(image_path, "rb") as image_file:
		encoded = base64.b64encode(image_file.read())
	return encoded
def prep_buffer(buffer):
	
	packet_size = 99999
	
	take_picture()
	rospy.loginfo("<-- PICTURE TAKEN -->")
	long_encoded = convertImageToBase64()
	end = packet_size
	start = 0
	length = len(long_encoded)
	pic_id = randomword(8)
	pos = 0
	no_of_packets = math.ceil(length/packet_size) + 1
	while start <= len(long_encoded):
		msg = Image(long_encoded[start:end], pic_id, pos, no_of_packets)
		buffer.append(msg)
		end += packet_size
		start += packet_size
		pos += 1 
		rospy.loginfo("<-- pos is -->" +  str(pos))
	return buffer


def talker():
	buffer = []
	pub = rospy.Publisher('images', Image, queue_size=10)
	#rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		buffer = prep_buffer(buffer)
		for msg in buffer:
			pub.publish(msg)
			#buffer.remove(msg)
			#rate.sleep()
		break



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    if data.data != "stop":
    
    	talker()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
