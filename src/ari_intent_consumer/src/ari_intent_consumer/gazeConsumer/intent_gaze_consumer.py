#!/usr/bin/env python3

from hri_actions_msgs.msg import Intent
import rospy
from intent_consumer import IntentConsumer
from hri_actions_msgs.msg import Intent
from ari_intent_publisher.intent_const import IntentConst

class GazeIntetntConsumer(IntentConsumer):

    def OnInit(self):
        pass
    
    def OnNotification(self, intent: Intent) -> bool:
        if intent.intent != IntentConst.ENGAGE_WITH:
            return False
        
        
