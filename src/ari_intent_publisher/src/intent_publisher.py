from hri_actions_msgs.msg import Intent
import rospy


class IntentPublisher:
    """ Wrapper class for creating a publisher that publishes to /intents.
    IntentPublisher abstracts away the process of creating/publishing data to /intents."""


    def __init__(self, name:str):
        """Initialise a publisher, named as ari_intent_publisher_arg."""
        self.__name = name
        self.__message = Intent()
        self.__publisher = rospy.init_node('ari_intent_publisher_' + self.__name)
        rospy.Publisher('/intents', Intent)

    def SetIntent(self, intent:str):
        """Set the intent of the message.
        Intent must be one of the constants from IntentConst."""
        self.__message.intent = intent
    
    def GetIntent(self)-> str:
        """Return the current intent"""
        return self.__message.intent
    
    def SetSource(self, source:str):
        """Set the source of the message.
        Source must be one of the constants from SourceConst."""
        self.__message.source = source
    
    def GetSource(self)-> str:
        """Return the current source"""
        return self.__message.source
    
    def SetModality(self, modality:str):
        """Set the modality of the message.
        Modality must be one of the constants from ModalityConst"""
        self.__message.modality = modality
    
    def GetModality(self)-> str:
        """Return the current modality"""
        return self.__message.modality

    def SetPriority(self, priority:int):
        """Set the priority of the message. This field is optional
        priority must be ranged between 0 ~ 255."""
        self.__message.priority = priority
    
    def GetPriority(self)-> int:
        """Return the current priority"""
        return self.__message.priority

    def ClearMessage(self):
        """Wipe out the message and start over."""
        self.__message = Intent()
    
    def SetConfidence(self, confidence:float):
        """Set the confidence of the message.
        Confidence must be ranged between 0.0 ~ 1.0."""
        self.__message.confidence = confidence
    
    def GetConfidence(self)-> float:
        """Return the current confidence"""
        return self.__message.confidence
    

    ### To do: Write functions for string data (map<str, str>)




    class IntentConst:
        """Definitions of constant variables for /intents"""
        ENGAGE_WITH="__intent_engage_with__"
        MOVE_TO="__intent_move_to__"
        GUIDE="__intent_guide__"
        GRAB_OBJECT="__intent_grab_object__"
        BRING_OBJECT="__intent_bring_object__"
        PLACE_OBJECT="__intent_place_object__"
        GREET="__intent_greet__"
        SAY="__intent_say__"
        PRESENT_CONTENT="__intent_present_content__"
        PERFORM_MOTION="__intent_perform_motion__"
        START_ACTIVITY="__intent_start_activity__"
        STOP_ACTIVITY="__intent_stop_activity__"
        

    class SourceConst:
        ROBOT_ITSELF="__myself__"
        REMOTE_SUPERVISOR="__remote_supervisor__"
        UNKNOWN_AGENT="__unknown_agent__"
        UNKNOWN="__unknown__"

    class ModalityConst:
        MODALITY_SPEECH="__modality_speech__"
        MODALITY_MOTION="__modality_motion__"
        MODALITY_TOUCHSCREEN="__modality_touchscreen__"
        MODALITY_OTHER="__modality_other__"
        MODALITY_INTERNAL="__modality_internal__"
    



    




