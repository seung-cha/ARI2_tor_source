#!/usr/bin/env python3

import rospy
from llama_chatbot.srv import *


if __name__ == "__main__":
    print("Waiting for llama_chatbot_response to be available")
    rospy.wait_for_service("llama_chatbot_response")
    llama = rospy.ServiceProxy("llama_chatbot_response", LlamaChatbotResponse)

    print("llama_chatbot_response is available!")
    try:
        while True:
            string = input("\nEnter prompt: ")
            response = llama(string)
            print("Response: " + response.response)
    
    except Exception as e:
        print("System stopped!")
        print(e)

