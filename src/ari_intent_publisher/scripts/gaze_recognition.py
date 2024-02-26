#!/usr/bin/env python3
"""
Detect face and start user engagement if face is detected for longer than 3 seconds.
Publish data to /intents and /gaze_target.

Robot is inherently unable to recognise faces (i.e ROS4HRI-compatible face detection publishes all person ID as anonymous).

Detection Strategy:
    Robot sees a face (or multiple faces) for >= 3 seconds.
    Robot initiates engagement and looks at the face until it is lost.
    When the face is lost, try to look for other faces. If there is one, start tracking that face instead.
    If no faces are present for 6 seconds, cease engagement, look front and repeat this process.

"""

import rospy
from pyhri.hri import HRIListener


from ari_intent_publisher.intent_publisher import IntentPublisher
from ari_intent_publisher.intent_const import IntentConst
from ari_intent_publisher.modality_const import ModalityConst
from ari_intent_publisher.source_const import SourceConst

from attention_manager.srv import SetPolicy
from std_srvs.srv import Empty


UpdateInterval = 0.1
IdleDuration = 10 * (1.0 / UpdateInterval)
RecognitionDuration = 3 * (1.0 / UpdateInterval)

class GazeDetector:

    def __init__(self):
        print('Initialising a node [1/2]')
        self.intent_pub = IntentPublisher('gaze_detection')
        self.intent_pub.SetIntent(IntentConst.ENGAGE_USER)
        self.intent_pub.SetModality(ModalityConst.MODALITY_VISUAL)
        self.intent_pub.SetSource(SourceConst.UNKNOWN_AGENT)

        # hri listener and [faceID, int] dictionary.
        # the integer corresponding to faceID increments gradually
        # to the point where the robot initiates engagement.
        self.hri = HRIListener('map')
        self.recogList = dict()

        # engagement target, empty indicates none.
        # robot will continuously look at this target.
        self.engagementTarget = ''
        self.engagementFrame = ''

        self.activeEngagement = False   # Is the robot currently engaged?
        self.idleDuration = IdleDuration           # How long does the robot look for faces when lost? (18 times, every 0.5 seconds = for 6 seconds)

        # subscribe to ari_gaze_manager. This service handles gaze for us.
        print('Subscribing to ari_gaze_manager [2/2]')

        rospy.wait_for_service('/attention_manager/set_policy')
        self.proxy = rospy.ServiceProxy('/attention_manager/set_policy', SetPolicy)
        self.reset_gaze = rospy.ServiceProxy('gaze_manager/reset_gaze', Empty)

        print(self.proxy(1, '')) 


        print('Node is ready!')



    def Update(self):
        #print('Updating')

        # Count down.
        self.idleDuration = self.idleDuration - 1




        # face is lost.
        if self.engagementTarget not in self.hri.faces and self.activeEngagement:
            #print('face is lost')
            self.engagementTarget = ''  # current engagement target doesn't exist. Try to look for a new face.
            self.engagementFrame = ''

        elif self.activeEngagement:
            self.idleDuration = IdleDuration     # reset the counter and look at the face
            
            self.proxy(3, self.engagementFrame)
            #print(self.proxy(3, self.engagementFrame))

        
        # Disable engagement
        if self.idleDuration <= 0 and self.activeEngagement:
            self.activeEngagement = False
            print('engagement disabled, publishing intent')
            self.intent_pub.SetIntent(IntentConst.DISENGAGE_USER)
            self.intent_pub.Publish()
            # reset gaze
            self.proxy(1, '')



        for id, face in self.hri.faces.items():


            #print("Currently seeing face %s" % id)

            # if engaged but tracking face is lost, simply replace it.
            if len(self.engagementTarget) == 0 and self.activeEngagement:
                print('new face found')
                self.engagementTarget = id
                self.engagementFrame = face.frame

                


            # Increment the counter
            if id in self.recogList:
                 self.recogList[id] =  self.recogList[id] + 1
            else:
                 self.recogList[id] = 1

            # Engage with this target only when not engaged and seen for long.
            if self.recogList[id] >= RecognitionDuration and not self.activeEngagement:
                print('engaging, changing gaze mode')
                self.engagementTarget = id
                self.engagementFrame = face.frame

                self.activeEngagement = True
                self.intent_pub.SetIntent(IntentConst.ENGAGE_USER)
                self.intent_pub.Publish()
                

            








if __name__ == '__main__':
    detector = GazeDetector()

    while not rospy.is_shutdown():
        detector.Update()
        rospy.sleep(UpdateInterval)