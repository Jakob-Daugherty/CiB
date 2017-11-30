# CiB
Currently in Beta - Submission environment

## Alfred

Created by: Matthew Romero Moore, Jakob Daugherty, and Kevin Free

### Requirements

- Raspberry Pi
- Pi SD Card with Debian installed
  - Instructions to install Debian from an image can be found [here](https://www.raspberrypi.org/documentation/installation/installing-images/README.md)
- iRobot Roomba and connecting cables

### Setup

1. Make sure that your pi is working and connected to the internet.
  - More detailed instructions on configuring your pi can be found [here](https://www.raspberrypi.org/documentation/configuration/)
2. Install the ROS working environment for robot integration
  - Follow the instructions provided [here](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi) to install ROS on your pi
  - Make sure you install the __Desktop__ version to enable robot integration.
3. Configure AWSIoT services for new thing (your pi)
  - For this you must have an AWS account. You can sign up for one [here](https://aws.amazon.com/free/?sc_channel=PS&sc_campaign=acquisition_US&sc_publisher=google&sc_medium=cloud_computing_b&sc_content=aws_account_bmm_control_q32016&sc_detail=%2Baws%20%2Baccount&sc_category=cloud_computing&sc_segment=102882724242&sc_matchtype=b&sc_country=US&s_kwcid=AL!4422!3!102882724242!b!!g!!%2Baws%20%2Baccount&ef_id=Wh9JjQAABFJvTARB:20171129235805:s)
  - You must also have a Cognito Pool ID. Instructions on setting up AWS Cognito and creating a pool ID can be found [here](http://docs.aws.amazon.com/cognito/latest/developerguide/create-new-user-pool-console-quickstart.html)
  - Follow the instructions provided by Amazon [here](http://docs.aws.amazon.com/iot/latest/developerguide/iot-sdk-setup.html)
  to get started with AWSIoT.
  - Additionally, you must obtain the generic AWS rootCA certificate when downloading your Thing's individual certificate.
  - Make sure to save the following items when configuring AWSIoT
    - Thing Endpoint
    - Thing Certificate and Public/Private key pair
    - Thing Policy name
    - Your AWS Region (e.g., us-west-2)
    - Your AWS Cognito Pool ID
4. Install and Configure Mosquitto
  - Instructions for installing Mosquitto on your pi can be found [here](https://aws.amazon.com/blogs/iot/how-to-bridge-mosquitto-mqtt-broker-to-aws-iot/)
  - Note: In these instructions, references to an AWS EC-2 instance should be replaced with your pi. Also mostly focus on the bridge.conf section, and how to configure MQTT client for connection to AWSIot
  - After this step, I would suggest using mosquitto_sub to test your MQTT connection. More detailed information on mosquitto_sub can be found [here](https://mosquitto.org/man/mosquitto_sub-1.html)
5. Import and setup catkin_ws
  - Copy and paste the catkin_ws folder of this repo, into the home folder for your pi.
  - invoke catkin_make and source the setup.bash file.
  - You may need to provide your own copies of the create autonomy driver. Source code can be found [here](https://github.com/AutonomyLab/create_autonomy)
  - You may need to provide you own copies of the mqtt_bridge libraries and rosbridge_suite. Source code can be found [here](https://github.com/RobotWebTools/rosbridge_suite)
6. Install and configure motion
  - Instructions for doing so can be found [here](http://htmlpreview.github.io/?https://github.com/Motion-Project/motion/blob/master/motion_guide.html)

### Running the robot side

To run the standard driver use the following code.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch alfred alfred.launch
```

To run the experimental driver use the following code.

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch alfred dev-alfred.launch
```

Note: if the above returns an error, your camera module may need to be enabled using ```sudo raspi-config``` or motion may be running in the background and can be stopped using ```sudo service motion stop```

### Running the app side

Open the .xcworkspace file in XCode, and follow the instructions located [here](https://github.com/jadppf/CiB/tree/master/Alfred_ios)
