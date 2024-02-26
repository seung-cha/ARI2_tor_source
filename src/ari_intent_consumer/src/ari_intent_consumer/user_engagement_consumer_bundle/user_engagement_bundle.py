#!/usr/bin/env python3

import rospy

from pal_interaction_msgs.msg import TtsAction, TtsGoal
from pal_device_msgs.msg import DoTimedLedEffectAction, DoTimedLedEffectGoal, LedEffectParams


from actionlib import SimpleActionClient


from ari_intent_consumer.intent_consumer import IntentConsumer
from hri_actions_msgs.msg import Intent

from ari_intent_publisher.intent_const import IntentConst
from ari_intent_publisher.modality_const import ModalityConst

# Load a number of consumers
from ari_intent_consumer.chatbotConsumer.intent_chatbot_consumer import ChatbotIntentConsumer


class UserEngagementBundle(IntentConsumer):
    """
    A bundle of consumers that activates/deactivates
    when IntentConst.ENGAGE_USER/DISENGAGE_USER is fired.
    """

    def OnInit(self):

        print('Initialising User Engagement Bundle')
        print('Waiting for TTS to be ready')

        self.isActive = False
        self.activeByTalk = False   #   Whether the engagement was done through vision or audio cue.

        self.ttsPub = SimpleActionClient('/tts', TtsAction)
        self.ttsPub.wait_for_server()

        self.ledPub = SimpleActionClient('/pal_led_manager/do_effect', DoTimedLedEffectAction)
        self.ledPub.wait_for_server()

        # Change the LED sensor, indicate that conversation is idle.
        msg = DoTimedLedEffectGoal()
        msg.devices = [0, 1, 2]
        msg.effectDuration.nsecs = 0

        msg.priority = 1
        msg.params.effectType = LedEffectParams.FIXED_COLOR
        msg.params.fixed_color.color.b = 1.0
        msg.params.fixed_color.color.a = 1.0
        self.ledPub.send_goal(msg)


        # Load a number of consumers.
        self.consumers = []
        self.consumers.append(ChatbotIntentConsumer())


        consumer:IntentConsumer
        for consumer in self.consumers:
            consumer.OnInit()



        print('TTS Ready!')

    def OnNotification(self, intent: Intent) -> bool:

        # Enable engagement
        if intent.intent == IntentConst.ENGAGE_USER and self.isActive is False:
            self.isActive = True

            # Record how it is initiated.
            # This info is necessary for disengaging conversation:
            # We do not want the robot to disengage because face is lost when it was initiated by talk.
            self.activeByTalk = intent.modality == ModalityConst.MODALITY_SPEECH

            # Give the user feedback
            msg = TtsGoal()
            msg.rawtext.lang_id = 'en_GB'
            msg.rawtext.text = 'Hi, How can I help you?'
            self.ttsPub.send_goal(msg)


            # Change the LED sensor, indicate that conversation is initiated.
            msg = DoTimedLedEffectGoal()
            msg.devices = [0, 1, 2]
            msg.effectDuration.nsecs = 0

            msg.priority = 1
            msg.params.effectType = LedEffectParams.FIXED_COLOR
            msg.params.fixed_color.color.g = 1.0
            msg.params.fixed_color.color.a = 1.0
            self.ledPub.send_goal(msg)


            print('Engaging!')
            return True
        
        # Disable engagement
        if intent.intent == IntentConst.DISENGAGE_USER and self.isActive is True:

            # If conversation was initiated by talk but try to disengage because loss of presence, do not disengage.
            if self.activeByTalk and intent.modality == ModalityConst.MODALITY_VISUAL:
                return False
            
            self.isActive = False

            # Give the user feedback
            msg = TtsGoal()
            msg.rawtext.lang_id = 'en_GB'
            msg.rawtext.text = 'Goodbye.'
            self.ttsPub.send_goal(msg)


            # Change the LED sensor, indicate that conversation is now terminated.
            msg = DoTimedLedEffectGoal()
            msg.devices = [0, 1, 2]
            msg.effectDuration.nsecs = 0

            msg.priority = 1
            msg.params.effectType = LedEffectParams.FIXED_COLOR
            msg.params.fixed_color.color.b = 1.0
            msg.params.fixed_color.color.a = 1.0
            self.ledPub.send_goal(msg)

            print('Disengaged')
            return False

        if self.isActive:

            consumer:IntentConsumer
            for consumer in self.consumers:
                consumer.OnNotification(intent=intent)


        return False
