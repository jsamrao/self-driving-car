

#!/usr/bin/env python
#####  IMPORTS   ######
from pygame.locals import *
import RPi.GPIO as GPIO
import PCA9685 as p
import time
import pygame
import numpy as np
import cv2
import os
import math
#######################

#####  INIT GPIO  #####

RIGHT_MOTOR_REV = 11
RIGHT_MOTOR_FORW = 12

LEFT_MOTOR_REV = 13
LEFT_MOTOR_FORW = 15
MOTOR_PINS = [RIGHT_MOTOR_REV, RIGHT_MOTOR_FORW, LEFT_MOTOR_REV, LEFT_MOTOR_FORW] 

# SERVO DRIVER
RIGHT_MOTOR_SPD = 4
LEFT_MOTOR_SPD = 5

global FORWARD, BACKWARD, LEFT, RIGHT
FORWARD  = True
BACKWARD = False
LEFT = False
RIGHT = True
global speed

class sdcar:
	
	global pwm
	# Slope threshold is a number between 0 and 1, e.g. 0.3. Because we will unlikely need to 
	# filter lines that have a slope greater than 1
	SLOPE_MIN_THRESHOLD = .3 #.3
	SLOPE_MAX_THRESHOLD = 100 
	LN_Y_INTERCEPT_THRESHOLD = 0 
	LN_X_INTERCEPT_MIN_THRESHOLD = 639
	LN_X_INTERCEPT_MAX_THRESHOLD = 0
	NEW_ANGLE_DETECT_THRESHOLD = 15
	TURN_ANGLE_FILTER_THRESHOLD = 10
	pwm = p.PWM()                    # For PWM IO
	GPIO.setwarnings(False)		 # Turn of wanrings
	GPIO.setmode(GPIO.BOARD)	 # Using actual pin numbers
	for pin in MOTOR_PINS:		 # Program these pins to be outputs
		GPIO.setup(pin, GPIO.OUT)	
	global pictureNum
	pictureNum =0
	WINDOW_WIDTH = 480
	WINDOW_HEIGHT = 320
	TOP_Y_CUTOFF = WINDOW_HEIGHT/2
	BOTTOM_Y_CUTOFF = WINDOW_HEIGHT-10
	WINDOW_MID = WINDOW_WIDTH/2

	loop_num = 0
    	foundLeftLine = False
        foundRightLine = False
	

#	cv2.namedWindow("w1", cv2.CV_WINDOW_AUTOSIZE)
	cv2.namedWindow("w1", cv2.WINDOW_NORMAL)
	#cv2.resizeWindow("w1", WINDOW_WIDTH, WINDOW_HEIGHT)
	camera_index = 0
	global capture
#	print "init video before"
	capture = cv2.VideoCapture(camera_index)
	capture.set(3,WINDOW_WIDTH)
	capture.set(4,WINDOW_HEIGHT)
	print "init video after"
	#global xLeft
	#global xRight
	leftLine = None
	rightLine = None
	currSteeringAngle = 0
	
	xLeftLineStartPt =0
	xLeftLineEndPt = 0
	
	xRightLineStartPt = 0
	xRightLineEndPt = 0
	
	LINE_THICKNESS = 2


	turnVectorAngle = 0
	currTurnAngle = 0
	turnAngle = 0

	angle0 = 0;
	angle1 = 1;
	angle2 = 2;
	angle3 = 3;

	turnDir = False
	RIGHT = False
	LEFT = True
	
	rightLineDetected = False
	leftLineDetected = True
        # Initial values:
        # left window, lower line start point x and y
	lw_ll = [0, WINDOW_HEIGHT, 0, 0 ]
        # left window, upper line start point x and y
	lw_ul = [WINDOW_MID, 0,0,0]
        # right window, lower line start point x and y
	rw_ll = [WINDOW_WIDTH/2, WINDOW_HEIGHT, 0,0]
        # right window, upper line start point x and y
	rw_ul = [WINDOW_WIDTH, WINDOW_HEIGHT,0,0]
		
	STEERING_ANGLE_SCALAR = 1
	def _init_(self):
		pwm.frequency = 60

	def setSpeed(self, spd):
		# Speed values on UI are between 0 and 100.
		# Motors have a counter that counts from 0 to 4095. We
		# configure a value in the chip between 0 and 4095 to
		# control the speed; with 0 = stop, 4095 = max speed.
		# So the mapping between UI and motor values are:
		# 0 -> 0, 100 -> 4095. So we will simply multiple the
		# UI value by 40 to get to the chip speed configuration 
		# value.

		# Write out the length of time period motors will be 
	 	# powered on (to control speed)
		pwm.write(LEFT_MOTOR_SPD, 0 ,40*spd)
		pwm.write(RIGHT_MOTOR_SPD, 0, 40*spd)
		if spd == 100:
			pwm.write(LEFT_MOTOR_SPD, 0 ,4095)
                	pwm.write(RIGHT_MOTOR_SPD, 0, 4095)		

	#move: gets the car moving in direction dir and at speed spd. 
	def move(self, dir, spd):
	
		self.setSpeed(spd)
		
		if dir == FORWARD:
			GPIO.output(RIGHT_MOTOR_REV, GPIO.HIGH)
			GPIO.output(RIGHT_MOTOR_FORW, GPIO.LOW)
			GPIO.output(LEFT_MOTOR_REV, GPIO.LOW)
			GPIO.output(LEFT_MOTOR_FORW, GPIO.HIGH)
		else:
			GPIO.output(RIGHT_MOTOR_REV, GPIO.LOW)
			GPIO.output(RIGHT_MOTOR_FORW, GPIO.HIGH)
			GPIO.output(LEFT_MOTOR_REV, GPIO.HIGH)
			GPIO.output(LEFT_MOTOR_FORW, GPIO.LOW)
		
	
	#stop: stops motors
	def stop(self):
		self.setSpeed(0)

	def turn(self, dir):
		if dir == LEFT:
			self.setTurnAngle(100)
		elif dir == RIGHT:
			self.setTurnAngle(-100)

	def setTurnAngle(self,angle):
		pwm.write(11,11,420+angle)
#pwm.write(11,11,433+angle)
		

	def setCameraAngle(self, angle):
		pwm.write(15, 15, 433+angle)

	def takePicture(self):
		os.system('fswebcam -r 960x720 -d /dev/video0 image.jpg')
		print "take picture fn"		
		img = cv2.imread("image.jpg")
		lines = self.processPicture(img)
		self.drawLines(lines,img)

	def processPicture(self,img):
		
		imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		edges = cv2.Canny(imgGray,50,150,apertureSize = 3)
		file.write(edges)
		file.close()
		lines = cv2.HoughLines(edges,1,np.pi/180,205)
		return lines

	def drawLines(self, lines,img):
		for r,theta in lines[0]:

   			 # Stores the value of cos(theta) in a
    			a = np.cos(theta)

    			# Stores the value of sin(theta) in b
    			b = np.sin(theta)

   			 # x0 stores the value rcos(theta)
   			x0 = a*r

 			   # y0 stores the value rsin(theta)
    			y0 = b*r

    			# x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
  			x1 = int(x0 + 1000*(-b))

    			# y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
    			y1 = int(y0 + 1000*(a))

    			# x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
    			x2 = int(x0 - 1000*(-b))


    			# y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
    			y2 = int(y0 - 1000*(a))

			cv2.line(img,(x1,y1), (x2,y2), (0,0,255),2)

		cv2.imwrite("imgLines.jpg", img)
	#	pictureNum = pictureNum + 1

	def checkDistance(self):
		TRIG = 37
		ECHO = 38

		GPIO.setup(TRIG, GPIO.OUT)
		GPIO.output(TRIG, 0)

		GPIO.setup(ECHO, GPIO.IN)
        	time.sleep(0.1)
	
	        print "starting measurement"
	
	        GPIO.output(TRIG, 1)
	        time.sleep(0.00001)
	        GPIO.output(TRIG,0)
	

        	while GPIO.input(ECHO) == 0:
        	        pass
        	start = time.time()
	
	        while GPIO.input(ECHO) == 1:
	                pass
	        stop = time.time()
	
	        print(stop-start) *17000, " cm"
		
		return (stop-start) *17000

#	def autoStop(self, distance):
		if distance <= 6:
			return True
		return False

	def calcLineSlope(self, x1, y1, x2, y2):
 		if (x2-x1) == 0:
			return True, 0 
		return False, (float(y2)-float(y1))/(float(x2)-float(x1))

	def calcLineSlopeFloat(self, x1, y1, x2, y2):
                if (x2-x1) == 0:
                        return True, 0
                return False, (float(y2)-float(y1))/(float(x2)-float(x1))

	def calcLineSlope2(self, x1, y1, x2, y2):
                if (x2-x1) == 0:
                        return  0
                return (float(y2)-float(y1))/(float(x2)-float(x1))

	
	def drawLine(self, x1, y1, x2, y2, frame):
		cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

 	# method takes in a line represented by a slope and a point on the line: x1, y1.
	# method returns the x coordinate of the point where the line intersects y = yCoordinate 
	def getXCoordForY(self,x1, y1, slope, yCoord):
		if slope == 0:
			return 0
		return int(((float(slope)*float(x1))+float(yCoord)-float(y1))/float(slope))
	
 	# method takes in a line represented by a slope and a point on the line: x1, y1.
	# method returns the y coordinate of the point where the line intersects x = xCoordinate 
	def getYCoordForX(self,x1, y1, slope, xCoord):
		if slope == 0:
			return 0
		return int((float(slope) * (float(xCoord) - float(x1))) + float(y1))
	
	#This method takes in start and end points of two lines and returns the coordinates 
	# of the intersection of the two lines. (x1, y1) and (x2, y2) are the start and end 
	# points of the first line. (x3, y3) and (x4, y4) are the start and end points of
	# the second line
	def calcIntersection(self, x1, y1, x2 , y2, x3, y3, x4, y4):
#		print x1, " ", y1, " ", x2, " ", y2, " ", x3, " ", y3, " ", x4, " ", y4
		isVerticalP, p = self.calcLineSlope(x1, y1, x2, y2)
		isVerticalQ, q = self.calcLineSlope(x3, y3, x4, y4)
#		print "slopes: ", p, ", ", q
		if isVerticalQ == False and isVerticalP == False:
			x = (float(y3)-float(y1)+ (float(p)*float(x1)) - (float(q)*float(x3)))/(float(p)-float(q))
			y = float(p)*(float(x) - float(x1)) + float(y1)
			return int(x), int(y)
		
	def DetectSteeringAdjustment(self, currAngle):
		self.angle3 = self.angle2
		self.angle2 = self.angle1
		self.angle1 = self.angle0
		self.angle0 = currAngle

		delta1 = abs(self.angle0 - self.angle1)
		delta2 = abs(self.angle1 - self.angle2)
		delta3 = abs(self.angle2 - self.angle3)

		#if delta1 < self.NEW_ANGLE_DETECT_THRESHOLD and delta2 < self.NEW_ANGLE_DETECT_THRESHOLD and \
		#   delta3 < self.NEW_ANGLE_DETECT_THRESHOLD:
		if delta1 < self.NEW_ANGLE_DETECT_THRESHOLD and delta2 < self.NEW_ANGLE_DETECT_THRESHOLD:
			return True
		else:
			return False
	
	def calcMidpoint(self, x1, y1, x2, y2):
		
		return (abs(x2-x1)/2) , (abs(y2-y1)/2)	

	def selfDrive(self):
	#	print "capture"
		#capture frame from webcam
        	ret, frame = capture.read()
	#	print "after capture"
		#Quit on bad frame
        	if ret != True:
        	        print 'bad frame'
        	        return
		#Convert image to grayscale
        	imgGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		
		#Take gray image and use canny function to dectect edges		
        	edges = cv2.Canny(imgGray,50,150,apertureSize = 3)

		#Detect all lines in the edges using HoughLinesP
        	#lines = cv2.HoughLinesP(edges,1,np.pi/180,100, 50, 20)
		#lines = cv2.HoughLinesP(edges,1,np.pi/180,60, 15, 10)
		lines = cv2.HoughLinesP(edges,1,np.pi/180, 50, 50, 1)


#		if self.leftLineDetected == True and self.rightLineDetected == True:
#			self.setTurnAngle(0)


		        # Initial values:
	        # left window, lower line start point x and y
        	self.lw_ll = [0, self.WINDOW_HEIGHT, 0, 0 ]
        	# left window, upper line start point x and y
        	self.lw_ul = [self.WINDOW_MID, 0,0,0]
        	# right window, lower line start point x and y
        	self.rw_ll = [self.WINDOW_WIDTH/2, self.WINDOW_HEIGHT, 0,0]
        	# right window, upper line start point x and y
        	self.rw_ul = [self.WINDOW_WIDTH, self.WINDOW_HEIGHT,0,0]

		self.lw_ll_detected = False
		self.lw_ul_detected = False
		self.rw_ll_detected = False
		self.rw_ul_detected = False

		#Filter, ....
		#Traverse through the lines detected 
#		self.foundLeftLine = False
#		self.foundRightLine = False
        	if lines is not None:
	#		print "line not none"
			
			###for leftWindow in range(0,self.WINDOW_WIDTH/2):
			###rightWindow = self.WINDOW_WIDTH-leftWindow - 1
	                for x1, y1, x2, y2 in lines[0]:
				#print "leftWindow", leftWindow
				#Calculate slope
				isVertical, slope = self.calcLineSlope(x1, y1, x2, y2)
				cv2.line(frame, (x1, y1), (x2, y2), (0, 0 , 255), self.LINE_THICKNESS)
				#x_intercept = self.getXCoordForY(x1, y1, self.calcLineSlope2(x1,y1,x2,y2), 0)
				#y_intercept = self.getYCoordForX(x1, y1, self.calcLineSlope2(x1,y1,x2,y2), 0)

				#Filter vertical lines and slopes that are less than threshold	
				#if abs(float(slope)) > self.SLOPE_MIN_THRESHOLD and \
				#   not(abs(float(slope) > self.SLOPE_MAX_THRESHOLD)) and not isVertical and \
				#   y_intercept > self.LN_Y_INTERCEPT_THRESHOLD and  \
				#   (x_intercept < self.LN_X_INTERCEPT_MIN_THRESHOLD or x_intercept > self.LN_X_INTERCEPT_MAX_THRESHOLD):
			
                                # Filter out unwanted lines
				if abs(float(slope)) > self.SLOPE_MIN_THRESHOLD and not isVertical:
                                    #cv2.line(frame, (x1, y1), (x2, y2), (255, 255 , 255), self.LINE_THICKNESS)        
                                    # If line is in the left window
                                    if x1 < self.WINDOW_MID: 
					#if x1 == leftWindow and self.foundLeftLine == False:
					###if x1 == leftWindow and x1 > self.lw_ll[0]:
					if x1 > self.lw_ll[0]:
						#print "leftwindow ll"
						
						#self.lw_ll[0] = x1							
						#self.lw_ll[1] = y1
						#self.lw_ll[2] = x2
						#self.lw_ll[3] = y2
						self.lw_ll[0:4:1] = [x1,y1,x2,y2]
						self.lw_ll_detected = True
						###self.leftLine = [x1,y1, x2, y2, self.calcLineSlope2(x1, y1, x2, y2)]
						###self.foundLeftLine = True
							
					###if x1 == leftWindow and x1 < self.lw_ul[0]:
					if x1 < self.lw_ul[0]:
						#self.lw_ul[0] = x1
						#self.lw_ul[1] = y1
						#self.lw_ul[2] = x2
						#self.lw_ul[3] = y2
                                                self.lw_ul[0:4:1] = [x1,y1,x2,y2]	
						self.lw_ul_detected = True
                                    # Right window 
                                    elif x1 > self.WINDOW_MID +65: 
				        #Slide Window in the left half of the screen. Starting from the leftkk
					#if x1 == rightWindow and self.foundRightLine == False:
					###if x1 == rightWindow and x1 <  self.rw_ll[0]:
					if x1 > self.rw_ll[0]:
						#self.rw_ll[0] = x1 
						#self.rw_ll[1] = y1 
						#self.rw_ll[2] = x2 
						#self.rw_ll[3] = y2
                                                self.rw_ll[0:4:1] = [x1,y1,x2,y2]

						self.rw_ll_detected = True
					#Save coordinates and slope of the left line in the array	
					###self.rightLine =  [x1,y1, x2, y2, self.calcLineSlope2(x1, y1, x2, y2)]
					###self.foundRightLine = True
					###if x1 == rightWindow and x1 > self.rw_ul[0]:
					if x1 < self.rw_ul[0]:
						#self.rw_ul[0] = x1
						#self.rw_ul[1] = y1
						#self.rw_ul[2] = x2
						#self.rw_ul[3] = y2
                                                self.rw_ul[0:4:1] = [x1,y1,x2,y2]

						self.rw_ul_detected = True

			
			if self.rw_ul_detected == True and self.rw_ll_detected == True:
				self.rightLineDetected = True
			if self.lw_ul_detected == True and self.lw_ll_detected == True:
				self.leftLineDetected = True			

			#draw the tiny lines ( lw_ll, lw_ul, rw_ll, rw_ul)
#			cv2.line(frame, (self.lw_ll[0], self.lw_ll[1]) , (self.lw_ll[2], self.lw_ll[3]), (255,255, 0), self.LINE_THICKNESS+1)
 #                       cv2.line(frame, (self.lw_ul[0], self.lw_ul[1]) , (self.lw_ul[2], self.lw_ul[3]), (255,255, 0), self.LINE_THICKNESS+1)
  #                      cv2.line(frame, (self.rw_ll[0], self.rw_ll[1]) , (self.rw_ll[2], self.rw_ll[3]), (0,255, 255), self.LINE_THICKNESS+1)
   #                     cv2.line(frame, (self.rw_ul[0], self.rw_ul[1]) , (self.rw_ul[2], self.rw_ul[3]), (0,255, 255), self.LINE_THICKNESS+1)

		
										
                        # Find intersection of left-window-lower-line with y = WINDOW_HEIGHT: Left-window-lower-point x coordinate
			lw_lpt = self.getXCoordForY(self.lw_ll[0], self.lw_ll[1],self.calcLineSlope2(self.lw_ll[0], self.lw_ll[1], self.lw_ll[2], self.lw_ll[3]), self.WINDOW_HEIGHT)
                        # Find intersection of left-window-upper-line with y = 0: Left-window-upper-point x coordinate
			lw_upt = self.getXCoordForY(self.lw_ul[0], self.lw_ul[1],self.calcLineSlope2(self.lw_ul[0], self.lw_ul[1], self.lw_ul[2], self.lw_ul[3]), 0)

                        # Line connecting the above two points is the left lane

                        # Find intersection of right-window-lower-line with y = WINDOW_HEIGHT: Right-window-lower-point x coordinate
			rw_lpt = self.getXCoordForY(self.rw_ll[0], self.rw_ll[1],self.calcLineSlope2(self.rw_ll[0], self.rw_ll[1], self.rw_ll[2], self.rw_ll[3]), self.WINDOW_HEIGHT)
                        # Find intersection of right-window-upper-line with y = 0: Right-window-upper_point x coordinate
			rw_upt = self.getXCoordForY(self.rw_ul[0], self.rw_ul[1],self.calcLineSlope2(self.rw_ul[0], self.rw_ul[1], self.rw_ul[2], self.rw_ul[3]), 0)

                        # Line connecting the above two points is the right lane

			# Draw the two lanes using the above four intersection points
                        # Left lane = left window lower point to upper point
                        # Right lane = right window lower point to upper point
			cv2.line(frame, (lw_lpt, self.WINDOW_HEIGHT) , (lw_upt, 0), (0, 255,0), self.LINE_THICKNESS)
 			cv2.line(frame, (rw_lpt, self.WINDOW_HEIGHT), (rw_upt, 0), (0,255,0), self.LINE_THICKNESS)
		
                        # Turn vector is from the bottom mid point of the screen/image to the mid point of the lw_upt and rw_upt points.
			# Find midpoint:
			###upperMidpointX, upperMidpointY  = self.calcMidpoint( rw_upt,0, lw_upt,0)
			###lowerMidpointX, lowerMidpointY = self.calcMidpoint(rw_lpt,self.WINDOW_HEIGHT,  lw_lpt, self.WINDOW_HEIGHT)
			###upperMidpointX = (rw_upt - lw_upt)/2
			upperMidpointX = (rw_upt + lw_upt)/2
                        upperMidpointY = 0
			###lowerMidpointX = (rw_lpt - lw_lpt)/2
			###lowerMidpointX = self.WINDOW_MID
			###lowerMidpointY = self.WINDOW_HEIGHT
			###upperMidpointY = 0
			###print  upperMidpointX, upperMidpointY
			###print lowerMidpointX, lowerMidpointY


			# Draw turn vector from image bottom middle point to the mid point of upper mid point calculated above
 			cv2.line(frame, (self.WINDOW_MID, self.WINDOW_HEIGHT), (upperMidpointX, upperMidpointY), (255, 0 , 0), self.LINE_THICKNESS)
			
			# Find turn vector angle
			###self.turnVectorAngle = math.degrees(math.atan2(abs(upperMidpointY-lowerMidpointY), abs(upperMidpointX - lowerMidpointX)))
			self.turnVectorAngle = math.degrees(math.atan2(abs(upperMidpointY - self.WINDOW_HEIGHT), abs(upperMidpointX - self.WINDOW_MID)))

                        # Set turn direction and calculate 
			if self.lw_ll_detected == True and self.lw_ul_detected == True and self.rw_ll_detected == True and self.rw_ul_detected == True:
				if (upperMidpointX - self.WINDOW_MID) > 0:
					self.turnDir = self.RIGHT
                                        self.currTurnAngle = 90-self.turnVectorAngle
                                        # Filter out glitches in currentTurnAngle
                                        makeSteeringAdjustment = self.DetectSteeringAdjustment(self.currTurnAngle)
                                        # We need to filter out the noise in currTurnAngle. So it has to be greater than a threshold to be taken
                                        # into account. Also there must not be any glitches. 
                                        if self.currTurnAngle > self.TURN_ANGLE_FILTER_THRESHOLD and makeSteeringAdjustment:
                                                self.turnAngle = self.currTurnAngle
					else:
                                                self.turnAngle = 0
					#turnAngle is the currSteeringAngle as direction is right
					self.currSteeringAngle = self.turnAngle+14
					#print self.currSteeringAngle
					#if self.currSteeringAngle > 40:
					#	self.currSteeringAngle = 40
					self.setTurnAngle(int(1.2*self.currSteeringAngle))
					#cv2.line(frame, (int(self.WINDOW_MID), int(self.WINDOW_HEIGHT)), (int(math.cos(self.currSteeringAngle)), int(math.sin(self.currSteeringAngle))), (255,255,255), self.LINE_THICKNESS)
#					print "Right turn: ", self.currSteeringAngle
                        	else:
                                        self.turnDir = self.LEFT
                                        self.currTurnAngle = 90-self.turnVectorAngle
                                        makeSteeringAdjustment = self.DetectSteeringAdjustment(self.currTurnAngle)
                                        if self.currTurnAngle > self.TURN_ANGLE_FILTER_THRESHOLD and makeSteeringAdjustment:
                                                self.turnAngle = self.currTurnAngle
					else:
                                                self.turnAngle = 0
					#print self.currSteeringAngle 
					#currSteeringAngle is negative of turnAngle as direction is left
					self.currSteeringAngle = -1*self.turnAngle-14	#-13
					#if self.currSteeringAngle < -40:
                                        #        self.currSteeringAngle = -40
                                        
					
					self.setTurnAngle(int(1.2*self.currSteeringAngle))
#					print "Left turn: ", self.currSteeringAngle
					#cv2.line(frame, (int(self.WINDOW_MID), int(self.WINDOW_HEIGHT)), (math.cos(self.currSteeringAngle), math.cos(self.currSteeringAngle)), (255,255,255), self.LINE_THICKNESS)
#			else: 
#				self.setTurnAngle(0)


                        #print "TURN ANGLE: ", self.turnVectorAngle, "CURRENT ANGLE: ", self.currTurnAngle, "STEERING ANGLE: ", self.turnAngle
                        #print "STEERING ANGLE: ", self.currSteeringAngle, "loop: ", self.loop_num
                        #self.loop_num = self.loop_num + 1
		
		cv2.imshow("w1", frame)
        	if cv2.waitKey(1) & 0xff == ord("q"):
 
       	        	capture.release
                	cv2.destroyAllWindows()




if __name__ == '__main__':
	car = sdcar()	
	pygame.init()
	screen = pygame.display.set_mode((100,100))
	loop = 0
	speed = 50
	car.setTurnAngle(0)
	car.setCameraAngle(0)
	loop = loop + 1
#	print "LOOP: ", loop
	#car.setSpeed(40)
#	start_ticks = time.time()
	#time.sleep(4)
#	while (time.time() - start_ticks) < 14:
	
		#car.selfDrive()
#		if (time.time() - start_ticks) > 8:	
#			car.move(FORWARD,40)
	car.setSpeed(0)
	secondsCounter = 0
	start_ticks = time.time()
	counter = 0
	totalFrames = 0
	pygame.font.init()
	myfont = pygame.font.SysFont('Comic Sans MS', 30)
	stopCounter = 0
	stopDemo = True

	while True:

		
		if time.time() - start_ticks >= 1:
			print "FPS: ",counter
		#	screen.fill(pygame.Color("black"))
                 #       textsurface = myfont.render("FPS: "+str(counter), False, (255, 255, 255))
			start_ticks = time.time()
		#	if counter > 1:
		#		secondsCounter = secondsCounter +1
		#		totalFrames = totalFrames + counter
				#print "AVG FPS: ", totalFrames/secondsCounter
		#		textsurface2 = myfont.render("AVG FPS: " + str(totalFrames/secondsCounter), False, (255, 255, 255))
		#		screen.blit(textsurface2,(0,50))	
                        counter = 0
		#	screen.blit(textsurface,(0,0))        
			stopCounter = stopCounter + 1
		counter = counter + 1
		#pygame.display.update()

		if stopCounter == 60:
			stopDemo = False

		speed = 45

		car.selfDrive()		

#distance = car.checkDistance()
#		if car.autoStop(distance)== True:
#			car.stop() 
		for event in pygame.event.get():
	                if event.type == pygame.KEYDOWN:
	        	        if event.key==K_a:
					car.setTurnAngle(-100)
				if event.key==K_d:
					car.setTurnAngle(100)
				if event.key==K_w:
					car.move(FORWARD,speed)
				if event.key==K_s:
					car.move(BACKWARD,speed)
				if event.key==K_RIGHT:
					if speed < 100:
						speed = speed + 10
					print "SPEED: ", speed
				if event.key==K_LEFT:
					if speed > 0:
						speed = speed - 10
					print "SPEED: ", speed
			if event.type == pygame.KEYUP:
				if event.key==K_a:
					car.setTurnAngle(0)
				if event.key==K_d:
					car.setTurnAngle(0)
				if event.key==K_w:
					car.stop()
				if event.key==K_s:
					car.stop()
	car.move(FORWARD, 0)
