#!/usr/bin/env python3
from hri_actions_msgs.msg import Intent
import rospy
from ari_intent_consumer.intent_consumer import IntentConsumer

from ari_intent_publisher.intent_const import IntentConst
from ari_intent_publisher.modality_const import ModalityConst
from ari_intent_publisher.source_const import SourceConst



from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from hri_actions_msgs.msg import Intent
from llama_chatbot.srv import *


class ChatbotIntentConsumer(IntentConsumer):



    def OnInit(self):
        print('llama consumer initialising')

        rospy.wait_for_service('llama_chatbot_response')
        self.llama = rospy.ServiceProxy('llama_chatbot_response', LlamaChatbotResponse)

        self.tts_server = SimpleActionClient('/tts', TtsAction)
        self.tts_server.wait_for_server()

        print('llama consumer initialised')



    
    def OnNotification(self, intent: Intent) -> bool:

        print('Llama node heard intent')
        print(intent.intent)
        print(intent.modality)
        print(intent.source)

        if intent.intent != IntentConst.ANSWER_CONTENT:
            return False
        
        if intent.modality != ModalityConst.MODALITY_SPEECH:
            return False
        
        if intent.source != SourceConst.UNKNOWN_AGENT:
            return False
        

        response = self.llama(intent.data)
        

        goal = TtsGoal()
        goal.rawtext.text = response.response
        goal.rawtext.lang_id = 'en_GB'

        self.tts_server.send_goal_and_wait(goal)


        return True


    
    

