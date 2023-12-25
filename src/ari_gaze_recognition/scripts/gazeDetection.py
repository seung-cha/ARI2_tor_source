#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient

from geometry_msgs.msg import PointStamped
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsText
from pyhri.hri import HRIListener


import tf

class GazeDetector:

    def __init__(self):

        print('Initialising a node [1/3]')

        rospy.init_node("Ari_HRI_Listener")

        self.hri = HRIListener('map')
        self.recogList = dict()
        self.tts = SimpleActionClient('tts', TtsAction)

        self.look = rospy.Publisher('/look_at', PointStamped, queue_size=10)

        print('Waiting for TTS server [2/3]')

        self.tts.wait_for_server()

        print('Node ready! [3/3]')

    def Update(self):
        print('Updating')

        for id, face in self.hri.faces.items():
            print("Currently seeing face %s" % id)

            if id in self.recogList:
                 self.recogList[id] =  self.recogList[id] + 1
            else:
                 self.recogList[id] = 1

            if  self.recogList[id] >= 3:
                print(f'{id} is staying for longer than 3 seconds')


                pub = PointStamped()
                pub.header.frame_id = 'map'
                t = face.transform('map')

                pub.point.x = t.transform.translation.x
                pub.point.y = t.transform.translation.y
                pub.point.z = t.transform.translation.z

                print(f'child: {t.child_frame_id}')


                self.look.publish(pub)

            


                self.Speak('Hi, how can I help you?')



        pass


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