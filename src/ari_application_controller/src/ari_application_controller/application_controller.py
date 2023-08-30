#!/usr/bin/env python3
from __future__ import annotations
import rospy
from hri_actions_msgs.msg import Intent
import ari_intent_consumer.intent_consumer as Intent_Consumer


# Python code that gathers intent consumers and runs them.

class ApplicationController:
    """DO NOT run rospy.init_node.
    It is NOT intended to use this class as a ROS node.
    Abstract class for ApplicationController.
    Create a class that inherits from this class
    and populate the _consumers list with IntentConsumer's"""
    def __init__(self):
        self._consumers = []
    
    def OnInit(self):
        """Called by ApplicationManager to initialise consumers."""
        for consumer in self._consumers:
            consumer.OnInit()
    

    def FireIntent(self, intent:Intent):
        """Called by ApplicationManager to fire intent."""
        for consumer in self._consumers:
            if consumer.OnNotification(intent=intent):
                break
    

    #These methods could be remove later.
    def AddConsumer(self, consumer:Intent_Consumer.IntentConsumer):
        self._consumers.append(consumer)
    
    def RemoveConsumer(self, consumer:Intent_Consumer.IntentConsumer):
        if consumer in self._consumers:
            self._consumers.remove(consumer)


