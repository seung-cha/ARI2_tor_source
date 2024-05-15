# TOR source code
Source code for my Taste of Research source code.

To run it make sure to export `LLAMA_FILE` that points to a llama model:

```
export LLAMA_FILE=${HOME}/dir1/dir2/.../dir3/model.gguf
```

Then, simply do:
```
roslaunch ari_application_manager manager.launch
```

# Setup

Make sure `ROS_IP`, `ROS_MASTER_URI` are setup and the workspace is sourced (as well as noetic)

```
export ROS_IP=...
export ROS_MASTER_URI='http://ari-XXc:11311`
source dir1/dir2/.../devel/setup.bash
```

# Description

Main functionality is to interact verbally with the robot via llama-powered chatbot system.

Robot engages if face is seen for >= 3 seconds or the word 'hello' is heard, disengages when face is lost or 'goodbye'.

It is very easy to include or exclude functionalities thanks to 
[intents](https://docs.pal-robotics.com/ari/sdk/23.1/development/intents.html). If time permitted fully functional multimodal interaction could have been possible.

# Required packages

* llama-cpp-py
* pyhri
* nltk python