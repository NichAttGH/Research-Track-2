from __future__ import print_function
import time
from sr.robot import *


#Instance of the class Robot
R = Robot()

#Global variables
a_th = 2.0 #Threshold for the control of the orientation
d_ths = 0.4 #Threshold for the control of the linear distance
d_thg = 0.6 #Threshold for the release of a silver block near a golden block
grapped_tokens = [] #Array to save the offset of blocks taken


#Drive the robot forward
def drive(speed, seconds):
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

#Make the robot turn
def turn(speed, seconds):
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

#Add blocks to the Array of blocks taken
def add_token_to_list(tokenOffSet):
    global grapped_tokens
    grapped_tokens.append(tokenOffSet)

#Check if the blocks in the robot's field of view have already been taken (True) or not (False)
def check_grapped_tokens(tokenOffSet):
    global grapped_tokens
    included=False
    for token in grapped_tokens:
            if (token==tokenOffSet):
                included=True
    return included

#Check the closest block never taken in the robot's field of view. If there are no "new" blocks, it returns -9
def check_closest_token(color):
    global grapped_tokens
    dist=200
    if (color=="gold"): #for golden blocks
        for token in R.see():
            if (token.dist < dist and token.info.marker_type==MARKER_TOKEN_GOLD and check_grapped_tokens(token.info.offset)==False):
                dist = token.dist
                angle = token.rot_y
                tokenOffSet = token.info.offset
    else: #for silver blocks
        for token in R.see():
            if (token.dist < dist and token.info.marker_type==MARKER_TOKEN_SILVER and check_grapped_tokens(token.info.offset)==False):
                dist = token.dist
                angle = token.rot_y
                tokenOffSet = token.info.offset
    if (dist==200):
        return -9,-9,-9
    else:
        return dist, angle, tokenOffSet


i=False #Variable to switch between searching for gold or silver blocks

# Store the start time
start_time = time.time()

while True:
    if (len(grapped_tokens)==6): #if the robot have taken all the 12 blocks, the program stops
        print("Goal achieved.")
        
        # Store the end time
	end_time = time.time()

	# Calculate the elapsed_time
	elapsed_time = end_time - start_time

	# Show start time, end time and elapsed time
	print("This is the start time: {}".format(start_time))
	print("This is the end time: {}".format(end_time))
	print("This is the elapsed time: {}".format(elapsed_time))
        
        exit()

    if (i==False): # switching between silver and golden blocks 
        dist, angle, tokenOffSet = check_closest_token("silver")
    else:
        dist, angle, tokenOffSet = check_closest_token("gold")

    if (dist==-9): #if there is no one block in the robot's field of view, the robot goes on turning right.
        print("The is no one block in the robot's field of view.")
        turn(20,0.2)

    elif (dist < d_ths and i==False): #if we are close to the silver token, we grab it.
        R.grab() 
        drive(-40,0.5)
        add_token_to_list(tokenOffSet)
        # print(len(grapped_tokens))
        # print(grapped_tokens)
        print("Gotcha!")
        turn(40,0.5)
        i=True

    elif (dist < d_thg and i==True): #if we are close to the golden token, we release the silver one.
        R.release()
        drive(-50,0.5)
        add_token_to_list(tokenOffSet)
        # print(len(grapped_tokens))
        # print(grapped_tokens)
        print("The robot is free.")
        turn(50,1)
        i=False

    elif (-a_th<= angle <= a_th): # if the robot is well aligned with the token, we go forward
        print("Going straight.")
        drive(70, 0.2)
    elif (angle < -a_th): # if the robot is not well aligned with the token, we move it on the left or on the right
        print("Turning left.")
        turn(-5, 0.2)
    elif (angle > a_th):
        print("Turning right.")
        turn(+5, 0.2)
