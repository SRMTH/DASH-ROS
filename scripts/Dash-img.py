#!/usr/bin/env python
import cv2
import numpy as np
import time
import rospy
from std_msgs.msg import String
y,u,v = 97,96,85
cap = cv2.VideoCapture(2)
x =360
flag1 = True
def getArea() :
	move="nothing"
	global x,y,flag1
	[y1,u,v] = [97,87,54]#[105, 117, 113]
	#flag1=True
	t=time.time()
	rec=True
	area1=0
	while rec:
		#rec,img = cap.read()
		#img= cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
		#cv2.imshow('image',img)
		boln,f = cap.read()
		img_yuv = cv2.cvtColor(f, cv2.COLOR_BGR2YUV)
		cv2.imshow("yuv",img_yuv)
		#cv2.imshow("g1",img_yuv)
		#img_yuv[:,:,2] = cv2.equalizeHist(img_yuv[:,:,2])
		#cv2.imshow("g2",img_yuv
		mask = cv2.inRange(img_yuv, (np.array([0,u-45,v-45])), (np.array([255,u+45,v+45])))
		cv2.imshow("Masking",mask)
		erode = cv2.erode(mask,None,iterations = 1)
		dilate = cv2.dilate(erode,None,iterations = 1)
		image,contour,hierarchy = cv2.findContours(dilate,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(img_yuv, contour, -1, (0,255,0), 2)
		if contour:
			cnt = contour[0]
			(x,y),radius = cv2.minEnclosingCircle(cnt)
		#center = (int(x),int(y))

		
		if x>340+90 :
			move = "right"

		elif x<300-90 :
			move = "left"

		else:

			if len(contour)>0:
				cnt = contour[0]
				area = int(cv2.contourArea(cnt))
                                print "area",area
				#print "area1",area1
				#print area-area1
				if flag1 and area<17000:
					move = "forward"
				else:
					flag1=False
					move = "backward"
				area1=area
		# print move
		if cv2.waitKey(1) == 27:
			break

                return move



def talker() :
	#msg=raw_input()
	pub=rospy.Publisher('get_area',String,queue_size=10)
	rospy.init_node('talker',anonymous=True) 
	rate=rospy.Rate(10)

	while not rospy.is_shutdown() :
		msg = getArea()
		pub.publish(msg)
                time.sleep(0.1)

if __name__=="__main__" :
	try :
		talker()
	except rospy.ROSInterruptException :
		pass
