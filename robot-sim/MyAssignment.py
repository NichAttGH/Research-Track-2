from __future__ import print_function

import time
from sr.robot import *

"""
First Assignment Attolino Nicholas
	What does the code do?
	There are 5 functions: drive, turn, find_silver, find_golden, checklist and each function is explained below;
	furthermore, 'while cycle' represents the steps that the robot must take to achieve the goal:
	to pair each SILVER token with each GOLDEN token.
	1) The first step represents the robot that initially look the closest SILVER token and tries to take it;
	2) Once taked, the program stores the offset of the SILVER token into the List 'tokens_taked' so that it no longer
	has to be considered among all the available SILVER tokens;
	2) After that, the robot look the closest GOLDEN token and release the SILVER token near to the GOLDEN token;
	3) At this point, the program stores the offset of the GOLDEN token into the List 'tokens_taked' so that it no longer
	has to be considered among all the available GOLDEN tokens;
	4) Now, the List has the first Pair and the program will continue to repeat each step until the last GOLDEN token
	because after the last, the program will warn us with a message that tells 'List Full, impossible to continue!'

"""
a_th = 2.0 # Threshold for the control of the orientation
d_th = 0.4 # Threshold for the control of the linear distance
R = Robot() # Instance of the class Robot
tokens_taked = [] # List of tokens taked and paired

def drive(speed, seconds): # Function for setting a linear velocity where speed(int) is the speed of the wheels
				# and seconds(int) is the time interval
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def turn(speed, seconds): # Function for setting an angular velocity where speed(int) is the speed of the wheels
				# and seconds(int) is the time interval
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0
    
def find_silver(): # Function to find the closest SILVER TOKEN that returns:
			# - dist(float) that is the distance of the closest token;
			# - rot_y(float) that is the angle between the robot and the token;
			# - offset that is the value that consider each token like unique in the list 'tokens_taked'
			# All of three variables return -1 if no token is detected!
    dist = 100
    for token in R.see():
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_SILVER and checklist(token.info.offset) == False:
            # In this condition, we check if the distance of the token is less than 100, if the token is SILVER
            # and if it isn't into the list
            dist = token.dist
	    rot_y = token.rot_y
	    offset = token.info.offset
    if dist == 100:
	return -1, -1, -1
    else:
   	return dist, rot_y, offset
   	
def find_golden(): # Function to find the closest GOLDEN TOKEN that returns:
			# - dist(float) that is the distance of the closest token;
			# - rot_y(float) that is the angle between the robot and the token;
			# - offset that is the value that consider each token like unique in the list 'tokens_taked'
			# All of three variables return -1 if no token is detected!
	dist = 100
	for token in R.see():
		if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD and checklist(token.info.offset) == False:
	    	# In this condition, we check if the distance of the token is less than 100, if the token is GOLDEN
            	# and if it isn't into the list
	    		dist = token.dist
	    		rot_y = token.rot_y
	    		offset = token.info.offset
	if dist == 100:
		return -1, -1, -1
    	else:
   		return dist, rot_y, offset
   	
def checklist(offset):	# Function that allows us to verify if an offset is into the list of tokens paired
	global tokens_taked
	for token in tokens_taked:
		if token == offset:
			return True
	return False

state = True # This variable just indicate if the token that we are searching is SILVER if True.
		# Initially, the robot search a SILVER TOKEN so the variable is set to true!

# Store the start time
start_time = time.time()

while 1:
	if len(tokens_taked) == 6: # This condition check if the list is full because if yes, the program ends.
		print("Is not possible to continue because the list of tokens paired is full..")

		# Store the end time
		end_time = time.time()

		# Calculate the elapsed_time
		elapsed_time = end_time - start_time

		# Show start time, end time and elapsed time
		print("This is the start time: {}".format(start_time))
		print("This is the end time: {}".format(end_time))
		print("This is the elapsed time: {}".format(elapsed_time))

		exit()
	
	if state == True: # Here, the robot look for the closest SILVER token, but if not found
				# it look for the closest GOLDEN token.
    		dist, rot_y, offset = find_silver()
    	else:
    		dist, rot_y, offset = find_golden()
    	    
    	if dist == -1: # Here, if the robot don't see any token, we rotate it.
    		print("I don't see tokens available!!")
        	turn(10,0.1)
        elif dist <= d_th and state == True: # Here the robot check if the current distance from the robot to the token is less 								# than d_th and if the token found is SILVER.
        	print("Found the Silver!")
        	R.grab() # If the robot is close to the token, it grab it.
        	tokens_taked.append(offset) # Here we add the SILVER token into the list
        	drive(-100,1)
        	state = not state # Here we modify the value of the variable to search the GOLDEN token
        elif dist <= d_th + 0.2 and state == False: # Here the robot check if the current distance from robot to the token is 								# less than d_th + 0.2 and if the token found is GOLDEN.
        	print("Found the Golden!")
        	R.release() # If the robot is close to the GOLDEN token, the robot release the SILVER token near to it
        	tokens_taked.append(offset) # Here we add the GOLDEN token into the list
        	print("Paired!")
        	drive(-25,1)
        	state = not state
        
        elif -a_th<= rot_y <= a_th: # If the robot is well aligned with the token, we go forward
        	print("Ah, here we are!.")
        	drive(200, 0.1)
        elif rot_y < -a_th: # If the robot is not well aligned with the token, we move it on the left or on the right
        	print("Left a bit...")
        	turn(-10, 0.1)
        elif rot_y > a_th:
        	print("Right a bit...")
        	turn(+10, 0.1)