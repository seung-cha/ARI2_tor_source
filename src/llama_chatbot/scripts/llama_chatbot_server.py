#!/usr/bin/env python3

import rospy
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

    llamaLLM = Llama(model_path="./models/ggml-model-q4_0.gguf", seed= -1, n_gpu_layers=35)

    msg = ChatCompletionMessage(role="User",content= '''You are in a vocal conversation with the user.
                                \nYour response must be clear and short. Your name is Ari.
                                \n Below is the trascript with the user.
                                \n\nUser: Hi, how are you doing?
                                \nYou: Hi! I'm doing great! Thanks for asking. Is there anything I can help you with?''')


    promptHistory = [msg]
    output = llamaLLM.create_chat_completion(promptHistory, max_tokens=-1)
    strOutput = output.get('choices')[0].get('message').get('content')
    print("Initial response: " + strOutput)

    
    print("\nHello World!")
    rospy.init_node("llama_chatbot_server")
    s = rospy.Service("llama_chatbot_response", LlamaChatbotResponse, GetResponse)

    rospy.spin()


