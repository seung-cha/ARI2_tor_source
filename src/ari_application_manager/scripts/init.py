#!/usr/bin/env python3

# Script to initialise ARI.

import rospy
from std_srvs.srv import Empty

from hri_msgs.msg import Expression
from attention_manager.srv import SetPolicy

if __name__ == '__main__':
    rospy.init_node('Init_ARI', anonymous=False)

    # Call service to gaze_manager/reset_gaze to make the robot face forwards.
    #rospy.wait_for_service('/gaze_manager/reset_gaze')
    reset_gaze = rospy.ServiceProxy('gaze_manager/reset_gaze', Empty)

    # Publish a message to /robot_face/expression to open the eyes.
    expression = rospy.Publisher('/robot_face/expression', Expression, queue_size=10)
    arg = Expression()
    arg.expression = Expression.NEUTRAL


    # Publish a message to /attention_manager/set_policy to control attention
    attention = rospy.ServiceProxy('attention_manager/set_policy', SetPolicy)

    rospy.sleep(1)
    
    expression.publish(arg)
    attention(2, '')
    reset_gaze()
    
    print("################################# INIT_COMPLETE #################################")


