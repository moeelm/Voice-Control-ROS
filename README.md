
# Voix
This package uses the Google Text-To-Speech API to allow for seamless human robot interaction by sending navigation commands issued by the user to robot. Main operating system used was ROS while Turtlebot and Gazebo were used for simulation. Actual experiments were conducted using RosAria and the Pioneer3AT robot.
# Why use Google API?
The goal is to create natural human robot interaction. Current voice processing software requires users to design speech recognition algorithms, collect datasets, as well as understand Natural Language Processing methods. The accuracy of these methods still does not exceed that of Google Cloud API given its massive datasets from the various voice services as well as the use of deep learning neural networks. By incorporating an accurate and robust voice processing software easily integrated with popular robotics software, users can include robust voice control into their robotics projects.
# Installation 

To use this software package, ensure you are running ROS for your respective system as well as have installed Google Cloud SDK (which is detailed below.)
However, we need to install PortAudio so we can use PyAudio to get mic data.
```bash
sudo apt-get install portaudio19-dev
```
Install all the requirements using pip by cloning the Github repo and installing all the packages in requirements.txt.

```bash
cd ~/catkin_ws/src
git clone https://github.com/moeelm/voice_ros.git
cd voice_ros
pip install -r requirements.txt
```

## Google Cloud Setup
Follow the instructions [here](https://cloud.google.com/speech/docs/quickstart) for configuring your Google Cloud project and installing the SDK for authentication. You will need a google/gmail account.

Usage of the Google Cloud SDK requires authentication. This means you require an API key and an activated service account to utilize the APIs.
 1. Setup a [service account](https://cloud.google.com/docs/authentication/getting-started)
 2. Download the service account key as a JSON.
 3. Check you have GOOGLE_APPLICATION_CREDENTIALS in your environment. This should be the path to the keys.
```bash
export GOOGLE_APPLICATION_CREDENTIALS='/path/to/key'
```
 4. Run the authentication command:
```bash
gcloud auth activate-service-account --key-file GOOGLE_APPLICATION_CREDENTIALS
```


# Usage
Follow the steps below to setup the package properly.

## Configure topics
Go into the config directory and change the following parameters in the `params.yaml` file:

* `results_topic`: (Optional) The topic where your results will be published.
* `project_id`: The name of your project for the Google Speech node. This is the name of your Google Cloud project when going through the Google Cloud setup.

## Launching nodes
To start the voice processing node, run the following command:
```bash
roslaunch voice_ros voice.launch
```
## Autonomous Navigation Using Turtlebot
![alt tag](http://github.com/moeelm/voice_ros/rviz-simul.png)

To run autonomous navigation using the Turtlebot robot as well as RVIZ simulator,
1. Launch Turtlebot as well as Gazebo environment
```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```
2. Run the map
```bash
rosrun map_server map_server /PATH/TO/myMap.yaml
```
3. Run the navigation demo. AMCL (Adaptive Monte Carlo Localization) is used for localizing the robot
```bash
roslaunch turtlebot_gazebo amcl_demo.launch
```
4. Launch ROS Vizualization (RVIZ)
```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```
The `2D Nav Goal` sends a navigation goal, `2D Pose Estimate` sets the pose of the robot.

# ROS Nodes

## mic_client
ROS node receives text from the Google Cloud Speech API and publishes it onto `text_topic` (see config/params.yaml). This is parsed and sent to the navigation node, `nav_v1.py`. `single_utterance` is set to true to allow for single user input. This prevents the voice processing from being continous, allowing the intent to be parsed and detected.

### Published Topics
`text_topic` ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))
Acquired text from the Google Cloud Speech API.
`text2_topic`([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))
Intent detected in voice command.

## Navigation
ROS node that takes text from the _mic\_client_ node, publishes the parsed intent and sends the spatial location associated with the intent to the `\move_base_simple/goal` topic. For example, `'microwave'`has the position coordinates `(2.00, -5.00, 0.000)` mapped to it.


