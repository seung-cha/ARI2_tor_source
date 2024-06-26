#!/usr/bin/env python3

import rospy
import os
from llama_cpp import *

from llama_chatbot.srv import *




def GetResponse(request):
    print("\n\nI received: " + request.prompt)
    prompt = "User: " + request.prompt
    msg = ChatCompletionMessage(role="User", content=prompt)
    promptHistory.append(msg)

    response = llamaLLM.create_chat_completion(promptHistory, max_tokens=-1, stop="User:")
    responseStr = response.get('choices')[0].get('message').get('content')
    promptHistory.append(ChatCompletionMessage(role="You", content=responseStr))

    print("I generated: " + responseStr)

    if len(promptHistory) >= 9:
        promptHistory.pop(1)
        promptHistory.pop(1)

    return responseStr


if __name__ == "__main__":

    try:
        llamaLLM = Llama(model_path=os.environ['LLAMA_FILE'], seed= -1, n_gpu_layers=32)

        msg = ChatCompletionMessage(role="User",content= '''Your name is Ari and you are in a conversation with a user.
                                    \nYour responses should be concise and clear. Try to keep your responses in 1 sentence.
                                    \nIf you do not understand the user's question or their question doesn't make sense, please ask them to repeat the question.''')


        promptHistory = [msg]
        output = llamaLLM.create_chat_completion(promptHistory, max_tokens=-1)
        strOutput = output.get('choices')[0].get('message').get('content')
        print("Initial response: " + strOutput)
    except Exception:
        print('Something went wrong with llama server. Please make sure you have an environment variable named LLAMA_FILE pointing to a llama model.')
        exit(1)

    
    print("\nLlama server says Hello World!")
    rospy.init_node("llama_chatbot_server")
    s = rospy.Service("llama_chatbot_response", LlamaChatbotResponse, GetResponse)

    rospy.spin()


