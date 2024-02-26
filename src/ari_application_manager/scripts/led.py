#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient
from pal_device_msgs.msg import DoTimedLedEffectAction, DoTimedLedEffectGoal, LedEffectParams


# led devices ids
BACK = 0
LEFT_EAR = 1
RIGHT_EAR = 2
RESPEAKER = 4

MAX_PRIORITY = 255  # The maximum value for leds effect priority


class LED:

    def __init__(self):
        rospy.init_node('led_test')

        self.pub = SimpleActionClient('/pal_led_manager/do_effect', DoTimedLedEffectAction)
        print('Waiting for the LED server.')
        self.pub.wait_for_server()

        self.ChangeLED()    # Change the LED

    def ChangeLED(self):
        msg = DoTimedLedEffectGoal()

        msg.devices = [BACK, LEFT_EAR, RIGHT_EAR, RESPEAKER]    # LEDs to modify.
        msg.priority = MAX_PRIORITY     # Set the priority level. Any data published with less priority will be ignored.

        msg.params.effectType = LedEffectParams.FIXED_COLOR # LED Mode. Refer to the official document for the list of params.   

        # Change the LED colour to green.
        # Note that you need to change the corresponding field of params.effectType.
        # For example, if params.effectType = BLINK, then you need to modify params.blink instead.
        msg.params.fixed_color.color.r = 0.0
        msg.params.fixed_color.color.g = 1.0
        msg.params.fixed_color.color.b = 0.0
        msg.params.fixed_color.color.a = 1.0

        # Display the new LED for 5 second. It will then go back to the previous LED colour
        # If effectDuration.secs = 0, LED will be set permanently (it will reset when the robot is turned off).
        msg.effectDuration.secs = 5

        self.pub.send_goal(msg)




if __name__ == '__main__':
    led = LED()
    print('Done')