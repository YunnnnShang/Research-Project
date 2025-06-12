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

## Getting Started
### Setup
#### Set up your Raspberry Pi 5
1. **Hardware Requirements:**
   - Raspberry Pi 5
   - Official Raspberry Pi Camera Module
   - Camera module cable
   - Power supply for Raspberry Pi 5
   - MicroSD card (with Raspbian OS installed)

2. **Ensure Sufficient Resources:**
   - Make sure you have a Raspberry Pi with sufficient resources. A Raspberry Pi 5 or later model with 8GB of RAM is recommended.

3. **Install 64-bit Operating System:**
   - Install the 64-bit operating system (e.g., Raspberry Pi OS). This ensures that your system can fully utilize the available hardware resources.

4. **Update the System:**
   - Ensure the Pi is up to date by using the following commands:
     ```sh
     sudo apt-get update
     sudo apt-get upgrade
     sudo apt update
     sudo apt install -y python3-libcamera python3-kms++ python3-picamera2
     ```

5. **Why 64-bit Instead of 32-bit?**
   - The main reason is because with a 32-bit OS, each process is limited to using 3GB of RAM, even if you have an 8GB memory card. With the 64-bit OS, a process can access all 8GB at once, making it more efficient for memory-intensive tasks.
### Installation

1. **Clone the repo:**
   ```sh
   git clone https://github.com/Prae-Flott/camera_degradation_raspberry.git
    ```
2. **Install Anaconda as described** [here](https://docs.anaconda.com/free/anaconda/install/index.html)
3. **Create an environment**
   with conda by putting this in your command line:
   ```sh
   conda env create -f yolov8_picam.yml
   ```
