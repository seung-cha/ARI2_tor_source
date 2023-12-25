#!/usr/bin/env python3

import rospy

from pal_interaction_msgs.msg import TtsAction, TtsGoal
from actionlib import SimpleActionClient


from ari_intent_consumer.intent_consumer import IntentConsumer
from hri_actions_msgs.msg import Intent

from ari_intent_publisher.intent_const import IntentConst

class UserEngagementBundle(IntentConsumer):
    """
    A bundle of consumers that activates/deactivates
    when IntentConst.ENGAGE_USER/DISENGAGE_USER is fired.
    """

    def OnInit(self):

        print('Initialising User Engagement Bundle')
        print('Waiting for TTS to be ready')

        self.isActive = False
        self.ttsPub = SimpleActionClient('/tts', TtsAction)
        self.ttsPub.wait_for_server()
        print('TTS Ready!')

    def OnNotification(self, intent: Intent) -> bool:

        if intent.intent == IntentConst.ENGAGE_USER:
            self.isActive = True

            # Give the user feedback
            msg = TtsGoal()
            msg.rawtext.lang_id = 'en_GB'
            msg.rawtext.text = 'Hi, How can I help you?'
            self.ttsPub.send_goal(msg)

            print('Engaging!')
            return True

        if self.isActive:
            pass


        return False
