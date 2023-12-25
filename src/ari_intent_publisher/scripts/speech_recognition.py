#!/usr/bin/env python3


import rospy
from hri_msgs.msg import LiveSpeech

from ari_intent_publisher.intent_publisher import IntentPublisher
from ari_intent_publisher.intent_const import IntentConst
from ari_intent_publisher.modality_const import ModalityConst
from ari_intent_publisher.source_const import SourceConst


class SpeechRecognition:

    def __init__(self):
        
        # Create an intent publisher to easily publish data to /intents
        self.intent_pub = IntentPublisher('speech_recognition')
        self.intent_pub.SetModality(ModalityConst.MODALITY_SPEECH)

        # Sub to speech recognition
        self.speechRecognition = rospy.Subscriber('/humans/voices/anonymous_speaker/speech',\
        data_class=LiveSpeech, callback=self.OnSpeechRecognised)

    
    def OnSpeechRecognised(self, data: LiveSpeech):
        speech = data.final

        if len(speech) == 0:
            return
        
        # If the robot hears the exact word 'Hello' or 'Bye', engage/disengage user.
        
        print(f'Heard: {speech}')

        if speech == 'hello':
            self.intent_pub.SetIntent(IntentConst.ENGAGE_USER)
            self.intent_pub.Publish()

            print('Intent Published: Engage')
        elif speech == 'goodbye':
            self.intent_pub.SetIntent(IntentConst.DISENGAGE_USER)
            self.intent_pub.Publish()

            print('Intent Published: Disengage')

        






if __name__ == '__main__':
    
    speechRecognition = SpeechRecognition()
    print('Speech recognition class intiailised')
    rospy.spin()



