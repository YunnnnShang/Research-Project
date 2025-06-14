# Investigating the Effects of Camera Degradations on the Performance of Selected Image Recognition Algorithms

![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![Pytorch](https://img.shields.io/badge/pytorch-blue?logo=pytorch)
![Static Badge](https://img.shields.io/badge/raspberry-purple?logo=raspberrypi)
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Passing Badge](https://img.shields.io/badge/Build-passing-green?style=for-the-badge)
![ROS Version](https://img.shields.io/badge/ROS%20version-humble-blue?style=for-the-badge)

## About The Project
Within the scope of a research project focused on predictive maintenance for distributed systems, a fleet of robots is currently being assembled as a demonstrator. Each robot, in addition to other sensors, is equipped with a Raspberry Pi 5, to which a camera is connected. This work initially aims to implement image processing algorithms on the Raspberry Pi, tailored for object detection, segmentation, or motion detection, considering the limited resources available. Furthermore, the algorithms will be trained to recognize other robots. An additional phase involves exploring how certain camera degradations might affect the performance of these image processing algorithms. These degradations must be induced either physically or through software manipulation. For a deeper understanding of the subject, [1] may be consulted. [1] Y. Pei, Y. Huang, Q. Zou, H. Zang, Z. Xy, und S. Wang, Effects of Im-age Degradations to CNN-based Image Classification. 2018.

### Software and Tools Used
Python 3.12
Anaconda 2023.07-1

## Getting Started: Setup and Installation
This project requires two distinct environments: a Raspberry Pi for data acquisition and robot control, and a Windows PC for data analysis and machine learning. Follow the setup instructions for both platforms.

### Set up your Raspberry Pi 5
The Raspberry Pi is used to interface with the iRobot Create 3, record raw sensor data into .mcap bag files, and perform initial pre-processing tasks.
#### A. System & Hardware Prerequisites
1. **Hardware Requirements:**
   - Raspberry Pi 5 (8GB RAM is recommended)
   - Official Raspberry Pi Camera Module & cable
   - Adequate power supply and MicroSD card
2. **Operating System:**
   - Install the 64-bit Raspberry Pi OS. This is essential for allowing processes to utilize the full memory capacity, which is critical for robotics and image processing tasks.
3. **Initial System Update:**
   - Ensure your system is up-to-date and has the necessary camera libraries installed:
     ```sh
      sudo apt-get update
      sudo apt-get upgrade -y
      sudo apt install -y python3-libcamera python3-kms++ python3-picamera2
     ```
#### B. ROS 2 Jazzy & iRobot Create 3 Installation
This is a corrected and unified procedure to install ROS 2 Jazzy and configure it for the iRobot Create 3.
1. **Set Locale:**
Ensure your system environment supports UTF-8. 
   ```sh
   locale  # check for UTF-8
   sudo apt update && sudo apt install locales -y
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   locale  # verify settings
    ```
2. **Add ROS 2 Repositories:**
Add the ROS 2 package sources to your system's package manager.
   ```sh
   # Enable the Ubuntu Universe repository
   sudo apt install software-properties-common -y
   sudo add-apt-repository universe -y
   
   # Add the ROS 2 GPG key
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   
   # Add the ROS 2 repository to your sources list
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```
3. **Install ROS 2 Jazzy Core Components:**
   ```sh
   sudo apt update
   sudo apt upgrade -y
   
   # Install ROS 2 Jazzy Desktop (includes RViz, demos, etc.)
   sudo apt install -y ros-jazzy-desktop
   
   # Install development tools (recommended)
   sudo apt install -y ros-dev-tools
   ```
4. **Install iRobot Create® 3 Specific Packages:**
These packages provide the necessary message definitions and tools for communicating with the Create 3 robot. 
   ```sh
   # Install Create 3 message definitions for Jazzy
   sudo apt install -y ros-jazzy-irobot-create-msgs

   # Install other recommended build and ROS tools for Jazzy
   sudo apt install -y build-essential python3-colcon-common-extensions python3-rosdep ros-jazzy-rmw-cyclonedds-cpp
   ```
5. **Configure Environment and Middleware (RMW):**
To have your environment ready automatically, add the ROS 2 sourcing command to your .bashrc file.
   ```sh
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   ```
6. **Set the default ROS 2 Middleware (RMW):**
This must match the RMW implementation on your robot (check its Application Configuration page).
   ```sh
   # Example for Fast RTPS (common default for Jazzy)
   echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
   
   # Or, if your robot uses Cyclone DDS:
   # echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
   ```
7. **Apply and Verify:**
Apply the configuration changes to your current terminal session:
   ```sh
   source ~/.bashrc
   ```
Verify: Connect your Raspberry Pi and Create 3 robot to the same network. Open a new terminal and run ros2 topic list. You should see a list of topics being published by the robot.
8. **Python Dependencies for Local Scripts:**
Install Python packages required for the data extraction scripts on the Pi.
   ```sh
   pip install pandas scipy
   ```
### Windows (PC) –  Data Analysis & ML Environment
The Windows PC is used for all heavy data processing, analysis, visualization, and machine learning tasks.
1. **Install Prerequisite Software:**
   - Install Anaconda. [here](https://docs.anaconda.com/free/anaconda/install/index.html)
   - Install Git.[here](https://git-scm.com/downloads)
2. **Clone the Project Repository:**
   ```sh
   git clone https://github.com/Prae-Flott/camera_degradation_raspberry.git
   cd camera_degradation_raspberry
    ```
3. **Prepare the yolov8_env Conda Environment**
   Two options to get a ready-to-use yolov8_env:
   ****A. Manually create the environment****
   ```sh
   # Create a new environment named 'yolov8_env' with Python 3.11
   conda create -n yolov8_env python=3.11 -y
   # Activate the new environment
   conda activate yolov8_env
   # Install Core Python Dependencies
   pip install ultralytics pandas opencv-python matplotlib seaborn scipy rosbags pybrisque
   ```
   ****B. Use the pre-built yolov8_env****
   Download yolov8_env.tar.gz from Releases.
   Extract into your Conda envs folder:
   ```
   # Linux/macOS example
   tar -xzf yolov8_env.tar.gz -C ~/anaconda3/envs/
   # Windows PowerShell example
   tar -xzf yolov8_env.tar.gz -C H:\anaconda3\envs\
   # Activate the new environment
   conda activate yolov8_env
   ```
By following these steps, you should have your environment set up and ready to run the project.

<!-- ROADMAP -->
## Roadmap
Descripition of the milestones of this project. Example:
- [x] Make the camera running: Show video stream on Raspberry Pi 5 using python (Felix task)
- [x] Install Yolov8 (ultralytics), get it running on the Pi5 (at first try nano version)
  - [x] Yolov8 object detection
  - [x] Yolov8 segmentation on raspberry Pi ([Link](https://medium.com/@elvenkim1/how-to-deploy-yolov8-segmentation-on-raspberry-pi-3a70470de231)
  - [x] Try different sizes of model (nano, small, medium, large, extra large)
     - [ ] Investigate ressource usage (use for example psutil python package). Make graphs:
        - [ ] FPS for each model sizehttps://ai.stackexchange.com/questions/17371/calculation-of-fps-on-object-detection-task
        - [ ] Performance for each model size on public dataset: https://docs.ultralytics.com/guides/yolo-performance-metrics/#conclusion
     - [ ] Evaluate performance on public dataset (KITTI or COCO)
- [ ] Transfer Learning: Train it to detect mobile robots create3 (you can choose which ones, use online datasets maybe, see https://docs.ultralytics.com/de/yolov5/tutorials/transfer_learning_with_frozen_layers/) 
- [ ] Attach soiled glass in front of camera, investigate effect on object detection. Scenario
   - [ ] Put robot on different places °. For each place:
      - [ ] One rotation of 360° without glass in front camera lens
      - [ ] One rotation of 360° with glass in front of camera lens


## Project Workflow
###
