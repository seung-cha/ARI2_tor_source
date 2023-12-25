#!/usr/bin/env python3

import rospy
import ari_intent_consumer.intent_consumer as IntentConsumer
from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from hri_actions_msgs.msg import Intent


class TestIntentConsumer(IntentConsumer.IntentConsumer):
    tts_client:SimpleActionClient
    def OnInit(self):
        print("Node Initiating, TestIntent")
        self.tts_client = SimpleActionClient('/tts', TtsAction) 
        self.tts_client.wait_for_server() 
        print("Node Initiated.")    
        

    def OnNotification(self, intent: Intent) -> bool:
        try:
            print("Sending goal")
            print('I heard an intent with ' + str(intent.intent) + ' intent.')
            goal = TtsGoal()
            goal.rawtext.text = 'I heard an intent with ' + str(intent.intent) + ' intent.'
            goal.rawtext.lang_id = 'en_GB'

            self.tts_client.send_goal(goal=goal)
            print("Goal sent")
            return False

        except rospy.ROSInterruptException:
            print("Unexpectedly finished")
            return False
    