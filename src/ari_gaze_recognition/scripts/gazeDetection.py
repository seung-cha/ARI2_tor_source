#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient

from geometry_msgs.msg import PointStamped
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsText
from pyhri.hri import HRIListener


from ari_intent_publisher.intent_publisher import IntentPublisher
from ari_intent_publisher.intent_const import IntentConst
from ari_intent_publisher.modality_const import ModalityConst
from ari_intent_publisher.source_const import SourceConst

import tf

class GazeDetector:

    def __init__(self):

        print('Initialising a node [1/3]')
        # pub to intent
        self.intentPub = IntentPublisher("gaze_detection")
        self.intentPub.SetModality(ModalityConst.MODALITY_OTHER)
        self.intentPub.SetSource(SourceConst.UNKNOWN_AGENT)


        self.hri = HRIListener('map')
        self.recogList = dict()
        self.tts = SimpleActionClient('tts', TtsAction)

        self.look = rospy.Publisher('/look_at', PointStamped, queue_size=10)

        print('Waiting for TTS server [2/3]')

        self.tts.wait_for_server()

        print('Node ready! [3/3]')

        self.active = False
        self.disengageCount = 3


    def Update(self):

        # If active and disengage reaches 0, stop being active.
        if self.disengageCount <= 0 and self.active:
            self.active = False
            print('engagement disabled, publishing intent')
            self.intentPub.SetIntent(IntentConst.DISENGAGE_USER)
            self.intentPub.Publish()





        self.disengageCount = self.disengageCount - 1

        for id, face in self.hri.faces.items():

            self.disengageCount = 3

            print("Currently seeing face %s" % id)

            if id in self.recogList:
                 self.recogList[id] =  self.recogList[id] + 1
            else:
                 self.recogList[id] = 1

            if  self.recogList[id] >= 3:

                #Activate engagement
                self.active = True

                print(f'{id} is staying for longer than 3 seconds')


                pub = PointStamped()
                pub.header.frame_id = 'map'
                t = face.transform('map')

                pub.point.x = t.transform.translation.x
                pub.point.y = t.transform.translation.y
                pub.point.z = t.transform.translation.z

                print(f'child: {t.child_frame_id}')


                self.look.publish(pub)

                self.intentPub.SetIntent(IntentConst.ENGAGE_USER)
                self.intentPub.Publish()

            




    def Speak(self, msg:str):

        print('Sending a TTS goal')
        goal = TtsGoal()
        goal.rawtext = TtsText()
        goal.rawtext.lang_id = 'en'
        goal.rawtext.text = 'Hi'

        self.tts.send_goal_and_wait(goal)
        print('goal sent!')


        pass




if __name__ == '__main__':
    detector = GazeDetector()

    while not rospy.is_shutdown():
        detector.Update()
        rospy.sleep(1.0)
    pass