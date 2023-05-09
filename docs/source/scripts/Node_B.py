#! /usr/bin/env python

"""
.. module:: Node_B
   :platform: Unix
   :synopsis: Python Node for the second assignment of RT1
   
.. moduleauthor:: Nicholas Attolino
  
This node implements the possibility for the user to see the number of
the goal reached from the robot and goal canceled from the user
  
Services:
    /Node_B

Subscribers:
    /reaching_goal/result

"""

# import ros stuff
import rospy
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from assignment_2_2022.srv import service_goals, service_goalsResponse

# Initialization of the 2 counters: count_goals_reached and count_goals_canceled
count_goals_reached = 0
count_goals_canceled = 0

def cb_results(message):    # This function sends the goals informations to the subscriber

    """
    Callback function to check the status of the robot to increment the corresponding counter

    """

    global count_goals_reached, count_goals_canceled

    sts = message.status.status
    
    if sts == 2:     # For cancelled goals is equals to 2
        count_goals_canceled += 1
    elif sts == 3:   # For reached goals is equals to 3
        count_goals_reached += 1

def data(req):  # This is the function of the service

    """
    Function of the Service that returns the value of both counters

    """

    global count_goals_reached, count_goals_canceled
    return service_goalsResponse(count_goals_reached, count_goals_canceled)


if __name__ == '__main__':

    """
    This function inizializes the ROS node 'Node_B',
    create the Service 'Node_B' to give to the user the number of
    goal reached and goal canceled and create a Subscriber for results

    """
   
    # Creation of the node B
    rospy.init_node('Node_B')

    # Creation of the service
    srv_count_goals = rospy.Service('Node_B', service_goals, data)

    # Creation of the subscriber for results
    subscriber_results = rospy.Subscriber('/reaching_goal/result', assignment_2_2022.msg.PlanningActionResult, cb_results)
    rospy.spin()
    