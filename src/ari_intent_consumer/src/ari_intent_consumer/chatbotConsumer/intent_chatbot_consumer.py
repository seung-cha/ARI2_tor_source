#!/usr/bin/env python3
from hri_actions_msgs.msg import Intent
import rospy
from ari_intent_consumer.intent_consumer import IntentConsumer

from ari_intent_publisher.intent_const import IntentConst
from ari_intent_publisher.modality_const import ModalityConst
from ari_intent_publisher.source_const import SourceConst



from actionlib import SimpleActionClient

from pal_interaction_msgs.msg import TtsAction, TtsGoal
from pal_interaction_msgs.srv import GetSpeechDuration, GetSpeechDurationRequest, GetSpeechDurationResponse

from hri_actions_msgs.msg import Intent
from llama_chatbot.srv import *

#Sentiment analysis
from nltk.sentiment.vader import SentimentIntensityAnalyzer

#robot face
from hri_msgs.msg import Expression

#Hutto, C.J. & Gilbert, E.E. (2014). VADER: A Parsimonious Rule-based Model for Sentiment Analysis of Social Media Text.
#Eighth International Conference on Weblogs and Social Media (ICWSM-14). Ann Arbor, MI, June 2014.

class ChatbotIntentConsumer(IntentConsumer):



    def OnInit(self):
        print('llama consumer initialising')

        rospy.wait_for_service('llama_chatbot_response')

        self.tts_server = SimpleActionClient('/tts', TtsAction)
        self.tts_server.wait_for_server()

        self.expression = rospy.Publisher('/robot_face/expression', Expression, queue_size=10)


        print('llama consumer initialised')

        goal = TtsGoal()
        goal.rawtext.text = "chatbot consumer is initialised."
        goal.rawtext.lang_id = 'en_GB'
        self.tts_server.send_goal(goal)


        self.time_wait = rospy.Time(secs=0)     # How long do I need to wait in order to  generate another response.
        self.llama = rospy.ServiceProxy('llama_chatbot_response', LlamaChatbotResponse)
        self.get_wait_time = rospy.ServiceProxy('get_speech_duration', GetSpeechDuration)

        self.vader = SentimentIntensityAnalyzer()

    def ChangeExpression(self, valence:float=0.0, arousal:float=0.0):
        data = Expression()
        data.valence = valence
        data.arousal = arousal

        self.expression.publish(data)


    
    def OnNotification(self, intent: Intent) -> bool:

        print('Llama node heard intent')
        print(intent.intent)
        print(intent.modality)
        print(intent.source)


        # Stop speaking if stop activity intent is received.
        if intent.intent == IntentConst.STOP_ACTIVITY:
            print('Stopping the current text to speech.')
            goal = TtsGoal()
            goal.rawtext.text = ' '
            goal.rawtext.lang_id = 'en_GB'
            self.tts_server.send_goal_and_wait(goal)
            return False


        # Only answer to modal:speech / intent:answer
        if intent.intent != IntentConst.ANSWER_CONTENT or intent.modality != ModalityConst.MODALITY_SPEECH:
            print('not answering.')
            return False
        

        # Get the expected response duration and stop the robot from responding during this time.
        # This is to avoid a loop where it continuously talks to itself.
        time_now = rospy.Time.now()

        if time_now >  self.time_wait:
            response = self.llama(intent.data)


            # Get sentiment data
            valence = self.vader.polarity_scores(response.response)['compound']
            print(f'Valence: ${valence}')

            if valence >= 0:
                self.ChangeExpression(valence=0.0, arousal=valence)
            else:
                self.ChangeExpression(valence=valence, arousal=0.0)

            

            goal = TtsGoal()
            goal.rawtext.text = response.response
            goal.rawtext.lang_id = 'en_GB'

            # Increment the wait time by expected duration of the speech. 
            delay_text = GetSpeechDurationRequest()
            delay_text.rawtext.lang_id = 'en_GB'
            delay_text.rawtext.text = response.response


            delay_time_response: GetSpeechDurationResponse
            delay_time_response = self.get_wait_time(delay_text)

            

            if  delay_time_response.ok:     # Assign a new incremented time to time_wait.

                delay_sum = rospy.Duration(secs=0)

                d: rospy.Duration
                for d in delay_time_response.word_durations:
                    delay_sum = delay_sum + d
                

                self.time_wait = delay_sum + time_now   # New wait time

                debug_diff = (self.time_wait - time_now) / 1e9
                print(f'New wait time is set: ${debug_diff}')            

            
        
            self.tts_server.send_goal_and_wait(goal)
            self.ChangeExpression()

            print('answered.')
            return True
        else:
            return False


    
    

