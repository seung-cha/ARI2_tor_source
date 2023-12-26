#!/usr/bin/env python3


import rospy
from pyhri.hri import HRIListener

from geometry_msgs.msg import PointStamped


from ari_intent_publisher.intent_publisher import IntentPublisher
from ari_intent_publisher.intent_const import IntentConst
from ari_intent_publisher.modality_const import ModalityConst
from ari_intent_publisher.source_const import SourceConst


class GazeDetector:

    def __init__(self):
        print('Initialising a node [1/2]')
        self.intent_pub = IntentPublisher('gaze_detection')
        self.intent_pub.SetIntent(IntentConst.ENGAGE_USER)
        self.intent_pub.SetModality(ModalityConst.MODALITY_OTHER)
        self.intent_pub.SetSource(SourceConst.UNKNOWN_AGENT)

        self.hri = HRIListener('map')
        self.recogList = dict()
        self.look = rospy.Publisher('/look_at', PointStamped, queue_size=10)

        print('Node ready! [2/2]')

        # To disengage activity
        self.active = False
        self.disengageCount = 3

    def Update(self):
        print('Updating')

        if self.disengageCount <= 0 and self.active:
            self.active = False
            print('engagement disabled, publishing intent')
            self.intent_pub.SetIntent(IntentConst.DISENGAGE_USER)
            self.intent_pub.Publish()

        self.disengageCount = self.disengageCount - 1

        for id, face in self.hri.faces.items():

            self.disengageCount = 3

            print("Currently seeing face %s" % id)

            if id in self.recogList:
                 self.recogList[id] =  self.recogList[id] + 1
            else:
                 self.recogList[id] = 1

            if self.recogList[id] == 3 and self.active is False:
                self.active = True
                self.intent_pub.SetIntent(IntentConst.ENGAGE_USER)
                self.intent_pub.Publish()



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

            








if __name__ == '__main__':
    detector = GazeDetector()

    while not rospy.is_shutdown():
        detector.Update()
        rospy.sleep(1.0)