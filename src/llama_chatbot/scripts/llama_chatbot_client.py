#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient
from llama_chatbot.srv import *
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from hri_msgs.msg import LiveSpeech



def connectTTS():
    tts = SimpleActionClient('/tts', TtsAction)
    tts.wait_for_server()
    return tts

def getSpeechMsg(msg):

    print("Receiving: " + msg.incremental)

    if len(msg.final) == 0:
        return
    
    print("Final: " + msg.final)

    response = llama(msg.final)
    print("Response: " + response.response)

    print("Requesting tts")
    goal = TtsGoal()
    goal.rawtext.text = response.response
    goal.rawtext.lang_id = 'en_GB'
    tts.send_goal_and_wait(goal)
    print("TTS successful.")

         



def connectSpeechRecognition():
    recog = rospy.Subscriber('/humans/voices/anonymous_speaker/speech', LiveSpeech, getSpeechMsg)



if __name__ == "__main__":
    rospy.init_node('llama_chatbot_client')
    print("Establishing connection to the TTS server")
    tts = connectTTS()
    


    print("Waiting for llama_chatbot_response to be available")
    rospy.wait_for_service("llama_chatbot_response")
    llama = rospy.ServiceProxy("llama_chatbot_response", LlamaChatbotResponse)

    print("llama_chatbot_response is available!")
    connectSpeechRecognition()
    rospy.spin()

    

