#!/usr/bin/env python3
import rospy
import sys
from ari_intent_publisher.intent_publisher import IntentPublisher
from ari_intent_publisher.intent_const import IntentConst
from ari_intent_publisher.modality_const import ModalityConst
from ari_intent_publisher.source_const import SourceConst
from hri_msgs.msg import LiveSpeech

# Hard-coded speech recognition that fires intent as it detects voice.
# Argument: A text file containing the list of hard coded voice lines and intent parameters, separated by comma (,)
# Each line consists of a hard coded string line followed by intent cost and optionally arguments names.
# Example:
#   I want you to take me to, GUIDE, place
# This indicates that for recognised line 'I want you to take me to #', fire a GUIDE intent with place = #.
#
# To do, in an unforeesable future: use intent classification to
# dynamically recognise intents


def OnSpeechReceived(data:LiveSpeech):
    speech = data.final
    if len(speech) == 0:
        return
    
    print("I heard: " + speech)

    for entry in commandSet:
        if entry['line'] in speech:
            # Get rid of the triggering line place blank
            speech = speech.replace(entry['line'] + ' ', '')
            speechArgs = speech.split(' ')

            #Hard-coded argument passing.
            args = dict()
            params = entry['args']
            for i in range(entry['len']):
                args[params[i]] = speechArgs[i]
            
            print('args:')
            print(args)

            #publisher.SetData(args)

            publisher.SetIntent(entry['intent'])
            publisher.PublishThenClear()
            return
    
    #Publish the data in hope to find a consumer that
    #can use this
    publisher.SetIntent(IntentConst.ANSWER_CONTENT)
    publisher.SetModality(ModalityConst.MODALITY_SPEECH)
    publisher.SetSource(SourceConst.UNKNOWN_AGENT)

    publisher.SetData(speech)

    publisher.PublishThenClear()
    print('published topic by default')





def ReadFile(filePath:str) -> (bool, list):
    magic = 'THIS IS A HARD-CODED SPEECH RECOGNITION FILE.'
    commandSet = list()

    try:
        file = open(filePath, "r")
        magicLine = file.readline().strip()

        #Validate the text file first
        if magicLine != magic:
            print("File does not contain the magic line!")
            return False, None
        
        for line in file:
            data = line.split(',')
            dataLen = len(data)
            #Each line must contain at least 2 lines.
            if dataLen < 2:
                print("File contains invalid line!")
                return False, None
            
            intentConst = IntentConst.ToIntentConst(data[1].strip())

            if intentConst == 'None':
                print("Invalid intent!")
                return False, None
            
            args = []

            for i in range(2, dataLen):
                args.append(data[i].strip())

            commandSet.append({
                'line': data[0].strip(), 
                'intent': intentConst,
                'len': dataLen - 2,
                'args': args})


        return True, commandSet
  
    except Exception as e:
        print(e)
        return False, None



if __name__ == '__main__':
    args = sys.argv
    n = len(args)

    if n != 2:
        print('Usage: ' + args[0] + ' [FILE_NAME]')
        sys.exit(0)
    
    status, commandSet = ReadFile(args[1])

    if status is False:
        sys.exit(0)


    publisher = IntentPublisher('voice_recognition')

    rospy.Subscriber(name='humans/voices/anonymous_speaker/speech', data_class=LiveSpeech, callback=OnSpeechReceived)
    print("Speech recog intent publisher initialised.")

    rospy.spin()
