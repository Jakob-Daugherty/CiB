import rospy
import picamera
from mqtt_video.msg import Image
from time import sleep
import base64
import random 
import string 
import math



def randomword(length):
	return ''.join(random.choice(string.lowercase) for i in range(length))


def take_picture():
	camera = picamera.PiCamera()
	try:
		camera.start_preview()
		sleep(1)
		camera.capture("image_test.jpg", resize=(640,480))
		camera.stop_preview()
		pass 
	finally:
		camera.close()
	
def convertImageToBase64():
	with open("image_test.jpg", "rb") as image_file:
		encoded = base64.b64encode(image_file.read())
	return encoded 


def prep_buffer(buffer):
	
	packet_size = 1000
	
	take_picture()
	rospy.loginfo("<-- PICTURE TAKEN -->")
	long_encoded = convertImageToBase64()
	
	end = packet_size
	start = 0
	length = len(long_encoded)
	pic_id = randomword(8)
	pos = 0 
	no_of_packets = math.ceil(length/packet_size)
	
	while start <= len(long_encoded):
		msg = Image(long_encoded[start:end], pic_id, pos, no_of_packets)
		buffer.append(msg)
		end += packet_size
		start += packet_size 
		pos = pos + 1 
	
	return buffer


def talker():
	
	
	
	packet_size = 1000
	
	buffer = []
	
    pub = rospy.Publisher('chatter', Image, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        
		buffer = prep_buffer(buffer)
		for msg in buffer:
        	pub.publish(msg)
        	buffer.remove(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        passReplace me
