#! /usr/bin/env python

"""
.. module:: Node_A
   :platform: Unix
   :synopsis: Python Node for the second assignment of RT1
   
.. moduleauthor:: Nicholas Attolino nicholasattolino@gmail.com
  
This node implements the possibility for the user to give a desired position
that the robot must reach
  
Publishes to:
    /position_and_velocity
    
Subscribes to:
    /odom
    
Clients:
    /reaching_goal

"""

# import ros stuff
import rospy
import actionlib
import actionlib.msg
import sys, select
import assignment_2_2022.msg
from nav_msgs.msg import Odometry
from assignment_2_2022.msg import custom_msg
from std_srvs.srv import *
from geometry_msgs.msg import Twist, Pose, Point

def action_client():
   
    """
    Function that create an action client, wait the server is active,
    take input from the user regards to the desired position that the robot must reach,
    create a goal for the robot and send it to the server

    """
    # Creation of the action client
    act_client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)

    # Waiting the server
    act_client.wait_for_server()

    while not rospy.is_shutdown():
        # Get input from the user
        while True:
            try:
                target_x = float(input('Coordinate X: '))
                target_y = float(input('Coordinate Y: '))
            except ValueError:
                print('Argh!! Please enter valid numbers \n')
                continue
            else:
                break

        # Creation of the goal for the robot
        goal = assignment_2_2022.msg.PlanningGoal()
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y

        # Send goal to the server
        act_client.send_goal(goal)

        # The user has 15 seconds to delete the goal by typing the word "cancel"
        print('Now you have 15 seconds to delete the goal')
        print('If you want to delete the goal, just write the word "cancel": ')
        word = select.select([sys.stdin], [], [], 15)[0]            # The select function is interesting
        if word:                                                    # because it allows us to take input
            value = sys.stdin.readline().rstrip()                   # from the user and set a timer within
            if (value == "cancel"):                                 # which the user can decide to cancel
                act_client.cancel_goal()                            # the target

def publish_msg(message):
   
    """
    Callback function to set the custom message that will be published

    Args:
    message(Pose)

    """
   
    global publisher
        
    # Get the position and velocity
    pos_x = message.pose.pose.position.x
    pos_y = message.pose.pose.position.y
    vel_x = message.twist.twist.linear.x
    vel_y = message.twist.twist.linear.y

    # Creation of the custom message
    custom_message = custom_msg()
    
    custom_message.actual_x = pos_x
    custom_message.actual_y = pos_y
    custom_message.actual_vel_x = vel_x
    custom_message.actual_vel_y = vel_y
    
    # Publishing the custom message
    publisher.publish(custom_message)

if __name__ == '__main__':

    """
    This function initializes the ROS node 'Node_A',
    control the robot publishing a custom message,
    waits for the robot's position and velocity from topic '/odom'
    and call the function 'action_client()'
    """

    # Creation of the node A
    rospy.init_node('Node_A')

    # Creation of the Publisher
    publisher = rospy.Publisher('/position_and_velocity', custom_msg, queue_size = 1)
    
    """
    Publisher for the robot' position and velocity
    """

    # Creation of the Subscriber
    subscriber = rospy.Subscriber('/odom', Odometry, publish_msg)
   
    """
    Subscriber for the current robot's position and velocity
    """
    
    action_client()
    