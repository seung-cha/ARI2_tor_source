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

        # Enable engagement
        if intent.intent == IntentConst.ENGAGE_USER and self.isActive is False:
            self.isActive = True

            # Give the user feedback
            msg = TtsGoal()
            msg.rawtext.lang_id = 'en_GB'
            msg.rawtext.text = 'Hi, How can I help you?'
            self.ttsPub.send_goal(msg)

            print('Engaging!')
            return True
        
        # Disable engagement
        if intent.intent == IntentConst.DISENGAGE_USER and self.isActive is True:
            self.isActive = False

            # Give the user feedback
            msg = TtsGoal()
            msg.rawtext.lang_id = 'en_GB'
            msg.rawtext.text = 'Goodbye.'
            self.ttsPub.send_goal(msg)

            print('Disengaged')
            return False

        if self.isActive:
            pass


        return False
