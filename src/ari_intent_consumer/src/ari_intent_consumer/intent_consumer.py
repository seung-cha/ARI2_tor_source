#!/usr/bin/env python3
from hri_actions_msgs.msg import Intent

class IntentConsumer:
    """Abstract class defined for consumer (subscriber) nodes."""
    """DO NOT call rospy.init_node()"""

    def OnInit(self):
        """Assume rospy.init_node() is called by ApplicationManager.
        Initialise ROS nodes here."""
        pass

    def OnNotification(self, intent: Intent) -> bool:
        """Called by ApplicationController when an intent is fired.
        Return False to pass the intent to other consumers and True to terminate.
        Even if this node does not need to do anything with the current intend, it is required to return a boolean."""
        pass
