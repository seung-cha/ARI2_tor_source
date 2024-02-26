#!/usr/bin/env python3
import rospy
from hri_actions_msgs.msg import Intent
import ari_application_controller.application_controller as Application_Controller
import ari_intent_consumer.chatbotConsumer.intent_chatbot_consumer as chatbot_consumer

from ari_intent_consumer.user_engagement_consumer_bundle.user_engagement_bundle import UserEngagementBundle



from std_srvs.srv import Empty

class ApplicationManager:
    def __init__(self):
        rospy.init_node(name= "ari_application_manager", anonymous=False)
        self.controller = Application_Controller.ApplicationController()
        self.intentSub = rospy.Subscriber('/intents', Intent, self.OnIntentReceived)
        
        #########################################3
        
        # Attach the starting bundle
        bundle = UserEngagementBundle()
        bundle.OnInit()

        self.controller.AddConsumer(bundle)
    

    def OnIntentReceived(self, data):
        """Called when an intent is published."""
        self.controller.FireIntent(data)




# Main ROS node that handles action controller and fires signal.

if __name__ == '__main__':
    manager = ApplicationManager()
    rospy.spin()



        

