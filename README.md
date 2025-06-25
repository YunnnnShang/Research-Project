# Investigating the Effects of Camera Degradations on the Performance of Selected Image Recognition Algorithms

![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![Pytorch](https://img.shields.io/badge/pytorch-blue?logo=pytorch)
![Static Badge](https://img.shields.io/badge/raspberry-purple?logo=raspberrypi)
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Passing Badge](https://img.shields.io/badge/Build-passing-green?style=for-the-badge)
![ROS Version](https://img.shields.io/badge/ROS%20version-humble-blue?style=for-the-badge)

## About The Project
Within the scope of a research project focused on predictive maintenance for distributed systems, a fleet of robots is currently being assembled as a demonstrator. Each robot, in addition to other sensors, is equipped with a Raspberry Pi 5, to which a camera is connected. This work initially aims to implement image processing algorithms on the Raspberry Pi, tailored for object detection, segmentation, or motion detection, considering the limited resources available. Furthermore, the algorithms will be trained to recognize other robots. An additional phase involves exploring how certain camera degradations might affect the performance of these image processing algorithms. These degradations must be induced either physically or through software manipulation. For a deeper understanding of the subject, [1] may be consulted.

[1] Y. Pei, Y. Huang, Q. Zou, H. Zang, Z. Xy, und S. Wang, Effects of Im-age Degradations to CNN-based Image Classification. 2018.

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
   ```sh
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
Descripition of the milestones of this project. 
- [x] Make the camera running: Show video stream on Raspberry Pi 5 using python (Felix task)
- [x] Install Yolov8 (ultralytics), get it running on the Pi5 (at first try nano version)
  - [x] Yolov8 object detection
  - [x] Yolov8 segmentation on raspberry Pi ([Link](https://medium.com/@elvenkim1/how-to-deploy-yolov8-segmentation-on-raspberry-pi-3a70470de231)
  - [x] Try different sizes of model (nano, small, medium, large, extra large)
     - [x] Investigate ressource usage (use for example psutil python package). Make graphs:
        - [x] FPS for each model sizehttps://ai.stackexchange.com/questions/17371/calculation-of-fps-on-object-detection-task
        - [x] Performance for each model size on public dataset: https://docs.ultralytics.com/guides/yolo-performance-metrics/#conclusion
     - [x] Evaluate performance on public dataset (KITTI or COCO)
- [x] Transfer Learning: Train it to detect mobile robots create3 (you can choose which ones, use online datasets maybe, see https://docs.ultralytics.com/de/yolov5/tutorials/transfer_learning_with_frozen_layers/) 
- [x] Attach soiled glass in front of camera, investigate effect on object detection. Scenario
   - [x] Put robot on different places °. For each place:
      - [x] One rotation of 360° without glass in front camera lens
      - [x] One rotation of 360° with glass in front of camera lens


## Project Workflow
### Step 1: Start SLAM and Navigation
1. connect to rpi
   ```sh
   ssh r5@<RPI_IP>
   cd ~/ws
   ```
2. Excecute on rpi:
   ```
   ws/start_slam_nav.sh
   ```
3. Publish the Initial Pose to localize the robot on the map. You must use a pose specific to your environment (e.g., office or lab).
   for **office**
   ```sh
   ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
     header: {
       stamp: {sec: 0, nanosec: 0},    # Zeitstempel 0 (Standard für initialpose)
       frame_id: "map"                # Standard-Frame "map"
     },
     pose: {
       pose: {
         position: {                  # Position am Ursprung
           x: -1.6,
           y: 0.3,
           z: 0.0
         },
         orientation: {               
           x: 0.0,
           y: 0.0,
           z: 0.0872,
           w: 0.9962                     
         }
       },
       covariance: [ # Kovarianz mit kleiner Unsicherheit (wie zuvor)
         0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
       ]
     }
   }'
   ```
   for **lab**:
   ```sh
   ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
     header: {
       stamp: {sec: 0, nanosec: 0},    # Zeitstempel 0 (Standard für initialpose)
       frame_id: "map"                # Standard-Frame "map"
     },
     pose: {
       pose: {
         position: {                  # Position am Ursprung
           x: 0.0,
           y: 0.0,
           z: 0.0
         },
         orientation: {               
           x: 0.0,
           y: 0.0,
           z: 0.0872,
           w: 0.9962                     
         }
       },
       covariance: [ # Kovarianz mit kleiner Unsicherheit (wie zuvor)
         0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
       ]
     }
   }'
   ```
4. (Optional) Visualize in RViz on PC:
   ```sh
   #On PC
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   export FASTRTPS_DEFAULT_PROFILES_FILE=~/ws/src/create3_examples/create3_republisher/dds-config/fastdds-passive-unicast.xml
   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
   ```
   
### Step 2: Start and Verify Camera Stream
1. On Pi, launch camera node at 1 FPS:
   ```sh
   #On Pi: Start Camera
   ros2 run camera_ros camera_node --ros-args -p camera:=0 -p role:=viewfinder -p width:=1920 -p height:=1080 -p FrameDurationLimits:="[1000000,1000000]" -p AeEnable:=false -p ExposureTime:=20000 -p AnalogueGain:=6.0
   ```
2. View image stream:
   ```sh
   #On Pi or on PC: Show image
   ros2 run rqt_image_view rqt_image_view /camera/image_raw _image_transport:=compressed
   ```
3. Check camera topics:
   ```sh   
   ros2 topic echo /camera/image_raw
   ros2 topic echo /camera/image_raw/compressed
   ```
### Step 3: Start Robot Driving
1. On Pi, configure DDS & stop daemon:
   ```sh
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   export FASTRTPS_DEFAULT_PROFILES_FILE=~/ws/src/create3_examples/create3_republisher/dds-config/fastdds-passive-unicast.xml
   ros2 daemon stop
   ```
2. Launch IR-based avoidance:
   ```sh
   ros2 run create3_teleop ir_avoider
   ```
### Step 4: Record Data
1. Start Recording:
   On the Pi, open a new terminal, source your ROS 2 environment, and use ros2 bag record to save all necessary topics into an .mcap file.(Ctrl+C to stop):
   ```sh
   ros2 bag record /odom /tf /camera/image_raw /cmd_vel /imu
   ```
2. Perform Experiment: Drive the robot through the desired scenario (e.g., a 360° rotation) with and without the degradation element (e.g., a soiled glass).
### Step 5: Extract Timestamped Image Frames (1 FPS) 
**Option A: ROS2 subscriber**
   ```sh
   # Terminal A: play bag
   cd ~/rosbagr5_20250514/rosbag2_<timestamp>
   source /opt/ros/humble/setup.bash
   ros2 bag play . --clock
   # Terminal B: extract frames
   dd ~/ws
   conda activate yolov8_env
   python live_frame_extractor.py
   ```
**Option B: Use the Batch Script**
On the Pi, use the batch_extract.sh script to process all your newly recorded .mcap files.
   ```sh
  # On Pi: Ensure paths in the script are correct and run it
  ./batch_scripts/batch_extract.sh
   ```
Output: The script calls extract_frames.py to create folders of images, with each image named using its precise ROS timestamp (e.g., frame_1748876155339538905.png).
### Step 5: Transfer Data to PC
Transfer MCAP Files: Copy the original .mcap files to your Windows PC (e.g., to H:\project_data\raw_mcaps\). These are required for extracting pose data. rsync is a great tool for this on Linux/macOS, or use any file transfer client like WinSCP on Windows.
   ```sh
   rsync -avz r5@<RPI_IP>:/home/r5/rosbagr5_20250514/ ~/rosbagr5_20250514/
   ```
### Step 6: Run YOLOv8 Object Detection
This phase takes place entirely on your Windows PC within your configured Conda environment (yolov8_env).
   ```sh
   # In Anaconda Prompt with yolov8_env activated
   python batch_scripts\batch_yolo_windows.py H:\project_data\processed_images windows_pc\run_yolov8_detection.py
   ```
Output: A yolo_detections_...csv file is generated in each image folder, containing object detection results for every frame.
### Step 7: Compute IQA, Extract Poses, and Merge Results
This is the core step where all data streams are unified.
1. Run the Consolidation Script: For each experimental run, execute process_and_merge_data.py. This script reads the YOLOv8 results, calculates BRISQE scores, reads the corresponding .mcap file to extract and synchronize poses, and merges everything.
   ```sh
   # Example for a single run
   python windows_pc\process_and_merge_data.py ^
       H:\project_data\processed_images\rosbagr5\r5_8 ^
       H:\project_data\raw_mcaps\rosbagr5\r5_8\bag_r5_8.mcap
   ```
Output: The final, unified final_ml_dataset.csv is generated in each run folder.
### Step 8: Final Comparative Analysis
Run the Analysis Script: Use final_analysis.py to load the final datasets from the runs you want to compare.
   ```sh
   # Example comparing three specific runs
   python windows_pc\final_analysis.py ^
       H:\...\processed_images\rosbagr5\r5_8\final_ml_dataset.csv ^
       H:\...\processed_images\rosbagr6\r6_7\final_ml_dataset.csv ^
       H:\...\processed_images\rosbagr7\r7_7\final_ml_dataset.csv ^
       --output_dir H:\project_data\final_reports
   ```
Output: The script prints summary statistics to the console and saves comparison plots to the specified output directory, allowing you to draw final conclusions about the impact of camera degradation.

<!-- Project Structure -->
## Project Structure
```text
camera_degradation_raspberry/
│
├── .gitignore                     # Specifies files and directories to be ignored by Git
├── LICENSE.txt                    # Your project's license file
├── README.md                      # The main project documentation file (this file)
│
├── raspberry_pi/                  # Scripts designed to run specifically on the Raspberry Pi
│   └── extract_frames.py          # Extracts timestamped image frames from .mcap files
│
├── windows_pc/                    # Scripts for data processing and analysis on the PC
│   ├── run_yolov8_detection.py      # Runs YOLOv8 inference on a single folder of images
│   ├── process_and_merge_data.py  # Calculates IQA, extracts poses, and merges all data
│   ├── final_analysis.py            # Analyzes final datasets and generates comparison plots
│   └── compare_specific_runs.py   # (Optional) For quick comparison of YOLOv8 results
│
├── batch_scripts/                 # Automation scripts to process multiple runs in batch
│   ├── batch_extract.sh             # (For Raspberry Pi) Automates running extract_frames.py
│   └── batch_yolo_windows.py      # (For Windows PC) Automates running run_yolov8_detection.py
│
├── data/                          # [NOT VERSIONED IN GIT] Local directory for all experimental data
│   ├── raw_mcaps/                 # Store original .mcap files from experiments here
│   └── processed_data/            # Store all generated data (frames, CSVs) here
│
├── reports/                       # [NOT VERSIONED IN GIT] Saved analysis reports and plots
│
└── environment/                   # Files for recreating the project's Python environment
    └── requirements.txt             # A list of Python dependencies for pip
```
Directory Explanations
raspberry_pi/ & windows_pc/: These folders separate the code based on the execution environment. raspberry_pi contains scripts that interface with ROS 2 native libraries, while windows_pc contains scripts for data-heavy analysis tasks.
batch_scripts/: Holds automation scripts to save time by processing large amounts of data without manual intervention.
data/: This is the main data directory, which should be kept local and not committed to Git. It's split into raw_mcaps for pristine data preservation and processed_data for all generated outputs.
models/: For storing the pre-trained YOLOv8 model weights you download. Also should be excluded from Git.
reports/: A place to save the final outputs of your analysis, such as the comparison plots, making it easy to find your results.
environment/: Contains files that define the necessary Python packages, allowing others (or your future self) to easily recreate the exact software environment needed to run the project.
