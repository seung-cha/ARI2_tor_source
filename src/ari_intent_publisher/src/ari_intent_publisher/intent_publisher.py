from hri_actions_msgs.msg import Intent
import rospy


class IntentPublisher:
    """ Wrapper class for creating a publisher that publishes to /intents.
    IntentPublisher abstracts away the process of creating/publishing data to /intents.
    DO NOT CALL rospy.init_node()
    """
    

    def __init__(self, name:str):
        """Initialise a publisher, named as ari_intent_publisher_arg."""
        self.__name = name
        self.__message = Intent()
        rospy.init_node('ari_intent_publisher_' + self.__name)
        self.__publisher = rospy.Publisher('/intents', Intent, queue_size= 10)

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

    def SetData(self, data:dict):
        """Set the data of the mssage.
        Data must be a dictionary.
        No specific form is required.

        AS OF NOW DATA IS STRING
        """
        self.__message.data = data
    
    def GetData(self) -> dict:
        """Return the current data

        AS OF NOW IT MAY RETURN A STRING INSTEAD
        """
        return self.__message.dataspeech


    def Publish(self):
        """Publish the current intent to /intents"""
        self.__publisher.publish(self.__message)

    def PublishThenClear(self):
        """Publish the current intent then wipe out the message"""
        self.Publish()
        self.ClearMessage()
    

    ### To do: Write functions for string data (map<str, str>)


    




