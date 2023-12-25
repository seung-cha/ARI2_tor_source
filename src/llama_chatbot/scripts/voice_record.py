#!/usr/bin/env python3
import wave
import rospy
from audio_common_msgs.msg import AudioData


AUDIO_RATE = 16000
AUDIO_CHANNELS = 1
AUDIO_WIDTH = 2


def channel_callback(msg, wf):
    wf.writeframes(msg.data)

if __name__ == '__main__':
    # call the relevant service
    rospy.init_node('record_audio')

    wf = wave.open("test_audio.wav", 'wb') #change output directory as desired
    wf.setnchannels(AUDIO_CHANNELS)
    wf.setsampwidth(AUDIO_WIDTH)
    wf.setframerate(AUDIO_RATE)

    #Record processed audio, corresponding to channel 0
    rospy.Subscriber('/audio/channel0', AudioData, channel_callback, wf)

    print("recording...")
    rospy.spin()
    print("saving...")
    wf.close()