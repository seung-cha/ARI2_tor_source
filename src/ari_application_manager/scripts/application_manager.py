#!/usr/bin/env python3
import rospy
from hri_actions_msgs.msg import Intent
import ari_application_controller.application_controller as Application_Controller
import ari_intent_consumer.testingConsumer.testing_intent_consumer as testing_intent_consumer

def OnIntentReceived(data):
    """Called when an intent is published."""
    controller.FireIntent(data)


# Main ROS node that handles action controller and fires signal.

if __name__ == '__main__':
    #Asign an application to controller
    controller = Application_Controller.ApplicationController()
    tempConsumer = testing_intent_consumer.TestIntentConsumer()
    tempConsumer.OnInit()

    controller.AddConsumer(tempConsumer)


    rospy.init_node(name= 'ari_application_manager', anonymous= False)
    rospy.Subscriber('/intents', Intent, OnIntentReceived)
    
    rospy.spin()
        

