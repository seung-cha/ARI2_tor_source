#!/usr/bin/env python3


"""
Gaze manager server. Given a faceID, rotate the robot's head so that it looks at the face.
invalid or empty faceID results in the robot looking front.

Service must be continuously requested in order for the robot to keep track of the face.
"""

import rospy

from geometry_msgs.msg import PointStamped
from attention_manager.srv import *     # Required to process incoming information
from ari_gaze_manager.srv import *      # Required for attention_manager/set_policy
from std_srvs.srv import *              # Required for gaze_manger/reset_gaze


from pyhri.hri import HRIListener

class GazeManager:
    
    def __init__(self):
        print('Initiating gaze_manager server')

        # Initialise the service node
        rospy.init_node('ari_gaze_manager_server')
        self.server = rospy.Service('gaze_target', GazeTarget, self.OnGazeRequested)


        # Wait for attention_manager service. This is to disable random gaze
        # which is the default behaviour of the robot.
        #print('Waiting for pal attention_policy to be ready')
        #rospy.wait_for_service('/attention_manager/set_policy')
        #self.attenProxy = rospy.ServiceProxy('/attention_manager/set_policy', SetPolicy)

        # Disable the natural gaze
        #print('attention_policy ready, disabling natural gaze')
        #self.attenProxy(0, '')

        # Wait for gaze_manager/reset_gaze. This is needed to reset gaze
        # when invalid faceID is received.
        #print('Waiting for pal reset_gaze to be ready')
        #rospy.wait_for_service('/gaze_manager/reset_gaze')
        #self.resetProxy = rospy.ServiceProxy('/gaze_manager/reset_gaze', Empty)
        #print('reset_gaze ready.')



        # Subscribe to the /look_at topic to control robot's gaze
        self.lookAtPub = rospy.Publisher('/look_at', PointStamped, queue_size=10)

        # Initialise HRI Listener
        self.hri = HRIListener('map')

        print('gaze_manager is ready to process information')


        

    
    def OnGazeRequested(self, data:GazeTargetRequest):
        print(f'Request received. parameter: {data.faceID}')

        point = PointStamped()

        # Make the robot look front by default
        point.header.frame_id = 'sellion_link'

        point.point.x = 10.0
        point.point.y = 0.0
        point.point.z = 0.0

        exist = False

        if data.faceID in self.hri.faces:
            print('faceID is valid!')
            face = self.hri.faces[data.faceID]
            t = face.transform('map')

            # face exists. Change the data to look at the person
            point.header.frame_id = '/map'
            point.point.x = t.transform.translation.x
            point.point.y = t.transform.translation.y
            point.point.z = t.transform.translation.z

            exist = True
        else:
            print('faceID is invalid')
            # Reset gaze
            self.resetProxy()

        self.lookAtPub.publish(point)
        return GazeTargetResponse(exist)





if __name__ == '__main__':
    gazeManager = GazeManager()
    rospy.spin()

