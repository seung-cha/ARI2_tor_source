#!/usr/bin/env python3
import rospy
from hri_actions_msgs.msg import Intent
import ari_application_controller.application_controller as Application_Controller
import ari_intent_consumer.testingConsumer.testing_intent_consumer as testing_intent_consumer
import ari_intent_consumer.chatbotConsumer.intent_chatbot_consumer as chatbot_consumer

from ari_intent_consumer.user_engagement_consumer_bundle.user_engagement_bundle import UserEngagementBundle

def OnIntentReceived(data):
    """Called when an intent is published."""
    controller.FireIntent(data)


# Main ROS node that handles action controller and fires signal.

if __name__ == '__main__':
    #Asign an application to controller
    rospy.init_node(name= 'ari_application_manager', anonymous= False)
    rospy.Subscriber('/intents', Intent, OnIntentReceived)
    
    controller = Application_Controller.ApplicationController()

    bundle = UserEngagementBundle()
    bundle.OnInit()
    controller.AddConsumer(bundle)


    
    rospy.spin()
        

