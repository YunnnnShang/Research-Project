This document outlines the comprehensive, end-to-end experimental plan for this research project. It details all phases from the initial setup to the final analysis, providing a clear roadmap for execution and tracking project milestones.

### Primary Objective: 
To systematically quantify and compare the performance impact of real-world camera degradation (induced by haze/soiling) on a general-purpose, pre-trained YOLOv8 model versus a domain-specific, fine-tuned YOLOv8 model.

### Phase 0: Foundation & Setup    
**Status:** ✅ Completed 

**Description:** The foundational work for all subsequent research has been successfully established.

**Checklist:**
  - [x] Successfully activated the camera and established a video stream on the Raspberry Pi 5.
  - [x] Installed and validated YOLOv8 (object detection and segmentation) on the Raspberry Pi 5.
  - [x] Conducted preliminary tests with various YOLOv8 model sizes (nano, small, etc.).
  - [x] Established a complete data pipeline from data acquisition on the Raspberry Pi (rosbag) to processing and analysis on a host PC.

### Phase 1: Baseline Performance Evaluation & Model Selection
**Status:** ✅ Completed

**Description:** Before investigating performance degradation, it is critical to establish accurate performance benchmarks under ideal conditions. This phase focuses on resource usage and performance benchmarking. The core goal is to find out which model size can run stably and efficiently in this restricted environment.

**Checklist:**
  - [x] Resource and Performance Benchmarking (on Raspberry Pi 5):\
        Objective: Test yolov8n, yolov8s, and yolov8m models on a clear, unobstructed video stream.\
        Metrics to Measure:\
            1. FPS: Real-time inference speed for each model size.\
            2. Resource Utilization: CPU and RAM usage during inference, measured using psutil.\
        Deliverable: Generate plots visualizing "Model Size vs. FPS" and "Model Size vs. Resource Utilization".
  - [x] Model Architecture Selection:\
        Objective: Based on the benchmark results, select one or two optimal model sizes for all subsequent degradation experiments.\
        Criteria: Balance inference speed, performance, and resource constraints. 
        Concluded from the benchmark data (e.g., yolov8n achieving only ~2.6 FPS) that yolov8n is the only model with minimal viability for CPU-based tasks, establishing it as the sole model for subsequent CPU-only experiments.

### Phase 2: Data Acquisition, Quantization & Dataset Construction
**Status:** `✅ Completed`

**Description:** Systematically create a well-structured and quantitatively-labeled dataset, which will be the foundation for the core experiments.

**Checklist:**
  - [x] Systematic Data Collection:\
        Scene Setup: Ensure the experimental environment contains both the **iRobot Create 3** (the specialist target) and a standard water bottle (the generalist target).\
        Condition 1 (Clear): Record `rosbag` files of the robot performing 360° rotations at multiple locations. This will serve as the Level 0 (clear) dataset.\
        Condition 2 (Degraded): Using varying levels of haze or soiled glass plates, record multiple `rosbag` sets (e.g., 3-4 distinct levels of degradation) under the same conditions as above.
  - [x] Data Processing and Quantization:\
        Execute scripts to extract all image frames from the `rosbag` files.\
        Calculate the BRISQUE score for every extracted frame.\
        Plot a histogram of all BRISQUE scores to visualize the distribution.
  - [x] Define Degradation Levels:\
        Based on the histogram, define formal thresholds for `Level 0, 1, 2, 3....` For example: `Level 0: BRISQUE < 35`, `Level 1: 35-50`, etc.
  - [x] Construct Leveled Datasets:\
        Write a script to automatically sort all images and their corresponding robot annotation files into leveled directories (`/data/leveled_datasets/level_0`, `/level_1`, etc.) based on their BRISQUE scores.\
        For each level, split the data into `train` (80%) and `val` (20%) subsets.

### Phase 3: Specialist Model Training
**Status:** `✅ Completed`

**Description:** Corresponds to the "Transfer Learning" milestone. This phase involves training a series of specialist models for the robot detection task.

**Checklist:**
  - [x] Train Baseline Model: Train `Model_A_0` using only the clean `Level 0 `training data. Save the best weights as `Model_A_0_best.pt`.
  - [x] Train Per-Level Models: For each degradation level (`Level 1, 2, 3...`), train a separate specialist model using its corresponding training data. Save the weights as `Model_A_1_best.pt`, `Model_A_2_best.pt`, etc.
  - [x] Train Mixed-Data Model: Combine the training sets from all levels to train a single, robust `Model_A_Mixed_best.pt`.
        
### Phase 4: Generalist Model Performance Evaluation
**Status:** `✅ Completed`

**Description:** The objective of this phase was to evaluate the performance of a pre-trained generalist model (yolov8n) on the prepared dataset to acquire the first set of core experimental results.

**Checklist:**
  - [x] Execute yolo val evaluation commands on all three degradation levels (Level 0, 1, 2).
  - [x] Obtained mAP scores for yolov8n detecting bottle across all levels, quantifying the performance degradation trend and log the results.\
    *Experimental Results*
    
    | Degradation Level |Physical Condition	|mAP50 Score|
    | --- | --- | --- |
    | Level 0	  | Clear / Control	  | 0.841 |
    | Level 1   | Haze Level 1  | 0.685 |
    | Level 2   | Haze Level 2  | 0.074 |

    *Key Conclusions*
    
    Baseline performance is robust: On clear images (Level 0), the yolov8n model performs well (mAP50 = 0.841), demonstrating the model's inherent capabilities.\
    Performance drops with degradation: After introducing mild blur (Level 1), the model performance drops significantly (by about 18.5%), but remains at a usable level.\
    Performance crash point: Under moderate blur (Level 2), the model performance drops off a cliff, with mAP50 dropping to 0.074, almost completely failing.\
    Preliminary argument: The experimental results strongly demonstrate that the reliability of standard pre-trained models decreases sharply when faced with real-world physical visual degradation, and this decrease may be nonlinear.

### Phase 5: Specialist Model Performance Under Degradation
**Status:** `✅ Completed`

**Description:** This phase fully characterized the specialist model's behavior under visual degradation through three key experiments:   1) A **direct evaluation** to measure its inherent robustness, 2) A **one-to-one fine-tuning** evaluation to measure its targeted adaptability, and 3) A **mixed-data fine-tuning** evaluation to create a single, general-purpose robust model.
**Checklist:**
  - [x] Established Model_R0 (trained on clear data) as the baseline model.:\
        Use the existing best.pt model to evaluate the performance on the Level 0 validation set. This will give us an exact baseline mAP score.
    
    *Experimental Results*
    
    | Degradation Level |Physical Condition	|mAP50 Score|
    | --- | --- | --- |
    | Level 0	  | Clear / Control	  | 0.983 |
    | Level 1   | Moderate Haze  | 0.995 |
    | Level 2   | Heavy Haze  | 0.951 | 
    | Level 3   | Severe Haze  | 0.412 | 
    
    *Key Conclusions*
    
    Specialized models perform well:On clear images (Level 0), the fine-tuned specialized model (mAP50=0.983) has significantly higher baseline performance than the general pre-trained model (mAP50=0.841).\
    Specialized models are extremely robust:Faced with moderate (New Level 1) and severe (New Level 2) image degradation, the performance of the specialized model barely drops (0.983 → 0.995 → 0.951), showing amazing resistance. This is in stark contrast to the general model, which begins to significantly degrade under mild degradation.\
    Specialized models have a performance inflection point:When image degradation reaches extremely severe (New Level 3), the performance of the specialized model eventually shows a significant collapse point, with mAP50 dropping sharply from 0.951 to 0.412. 

  - [x] Evaluated Model_R0 on all degradation levels (L0 to L3) to establish a performance degradation curve.
  - [x] Performed separate fine-tuning runs on each degradation level (L1 to L3), starting from Model_R0, to create adapted models. Evaluated each fine-tuned model on its corresponding degradation level.
       Created a final comparative analysis of "Direct Evaluation" vs. "Fine-Tuning" performance, quantifying the performance gain.

    *Experimental Results*

    The mAP@.50 scores clearly demonstrate the performance drop under direct evaluation and the subsequent recovery after fine-tuning.

    | Degradation Level | mAP50 (Direct Eval of Model_R0 | mAP50 (After Fine-Tuning)|Performance Gain |
    | --- | --- | --- | --- |
    | Level 0 (Clear) | 0.983 | N/A (Baseline) | N/A |
    | Level 1 (Moderate)   | 0.995 | 0.995 | 0.0% |
    | Level 2 (Heavy)  | 0.951 | 0.995 | +4.6% |
    | Level 3  (Severe)  | 0.412 | 0.995 | +141.5% | 

    *Key Conclusions*

    Inherent Robustness: The specialist model trained on clear data (Model_R0) demonstrated remarkable robustness against moderate and heavy degradation (Level 1 & 2), maintaining near-perfect performance.
    
    Performance Breaking Point: Under severe degradation (Level 3), the performance of Model_R0 collapsed, dropping from 0.951 to 0.412, proving that even specialist models have a clear failure threshold.
    
    Efficacy of Fine-Tuning: Fine-tuning proved to be an exceptionally effective strategy. For Level 3, it "rescued" the model from a near-failure state, restoring its performance to a near-perfect 0.995, a 141.5% relative improvement.

  - [x]  Created a comprehensive dataset by merging all levels, fine-tuned Model_R0 on it to create an "all-rounder" model, and evaluated its performance on all individual levels.
  - [x]  Created a final comparative analysis of all three training/evaluation strategies.

    **Final Results: Master Comparison Table** 

    The mAP@.50 scores for each strategy across all degradation levels are summarized below.
    
    | Degradation Level | mAP50 (Direct Eval) | mAP50 (One-to-One Fine-tune)| mAP50 ( Mixed-Data Fine-tune) |
    | --- | --- | --- | --- |
    | Level 0 (Clear) | 0.983 | N/A (Baseline) | 0.513 |
    | Level 1 (Moderate)   | 0.995 | 0.995 | 0.995 |
    | Level 2 (Heavy)  | 0.951 | 0.995 | 0.995 |
    | Level 3  (Severe)  | 0.412 | 0.995 | 0.995 | 
 
### Phase 6: Behavioral Analysis from MCAP Data
**Status:**  `In Progress`

**Description:**  To analyze the impact of camera degradation on the robot's physical driving behavior during an obstacle avoidance task. This serves as a "functional" or "system-level" assessment, complementing the direct image quality (IQA) and object detection (YOLOv8) analyses.

**Workflow:**
* Step 1: Raw Behavior Data Extraction (📍 on Raspberry Pi)

  The Raspberry Pi, with its native ROS 2 installation, was used as a pre-processing node to extract raw time-series data from the MCAP files into a portable format.
  
  Action: For each experimental run's `.mcap` file, a Python script was executed. Script Used: `extract_behavior_to_csv.py`
  
  Function: This script reads an `.mcap` file, subscribes to the `/odom` and `/cmd_vel` topics, and writes the entire message history for each into two separate CSV files.

* Step 2: Data Transfer (🍓 RPi → 💻 PC)

  The generated `odom_data.csv` and `cmd_vel_data.csv` files were transferred from the Raspberry Pi to their corresponding run folders on the Windows PC for the final analysis.

* Step 3: Behavioral Metric Calculation & Analysis (📍 on Windows PC)
  
  A comprehensive Python script was run to process the CSVs from all relevant runs. Script Used: `analyze_robot_behavior.py`
  
  Function: This script loads the `odom_data.csv` and `cmd_vel_data.csv` for each specified run and calculates the following key behavioral metrics:
  1.  **Stop Percentage / Hesitation:** The percentage of the total run time where the robot's commanded velocity was effectively zero.
  2.  **Total Path Length:** The total distance in meters covered by the robot, calculated from odometry data.
  3.  **Path Smoothness (RMS Jerk):** The Root Mean Square of the robot's jerk (the rate of change of acceleration), where a higher value indicates more erratic, less smooth motion.

**Key Findings & Interpretation**

The behavioral analysis yielded complex and insightful results, revealing a non-linear relationship between camera degradation and system-level performance.

* **Inconsistent Baseline:** The two control robots (`r6` and `r7`) exhibited significantly different baseline behaviors, with `r7` showing much higher natural hesitation (11.3% stop time) than `r6` (0.7%). This indicates that the obstacle avoidance algorithm has inherent variability.

* **Non-Linear Impact of Degradation:** Contrary to a simple hypothesis, increased camera blur did not lead to a linear increase in hesitant or erratic behavior.
    * The **mildly and moderately degraded** runs (`r5_8`, `r5_9`) showed **zero hesitation**, appearing paradoxically more "confident" than the control robots.
    * The **most jerky and erratic** run (`RMS_Jerk` = 10.4) was the one with the **least degradation** (`r5_8`).
    * The **most severely degraded** run (`r5_10`) was the only one in the experimental group to show significant hesitation (4.13%), suggesting a performance cliff or a change in failure mode once degradation becomes critical.

* **Core Conclusion:** The impact of sensor degradation on a robot's behavior is not straightforward. Mild degradation might "simplify" the world for an algorithm by removing visual noise, leading to seemingly bolder actions. This highlights that evaluating system performance requires a multi-faceted approach, as direct task performance (like YOLOv8) and system behavior can tell different, sometimes conflicting, stories.

---

## Required Scripts for this Phase

To replicate this phase of the experiment, the following two scripts are required:

1.  **`extract_behavior_to_csv.py`**
    * **Purpose:** To be run on the Raspberry Pi. Extracts time-series data from `/odom` and `/cmd_vel` topics within an `.mcap` file and saves them as `odom_data.csv` and `cmd_vel_data.csv`.
2.  **`analyze_robot_behavior.py`**
    * **Purpose:** To be run on the Windows PC. Takes multiple run folders (each containing the CSVs from the previous step) as input, calculates the behavioral metrics (Hesitation, Path Length, Jerk), and outputs a comparative summary table.











### Phase 7: Analysis, Visualization & Reporting
**Status:** `☐ To-Do`

**Description:** Transform the raw data into meaningful insights, visualizations, and conclusions.

**Checklist:**
  - [x] Generate Key Visualizations:\
        Create a heatmap for the performance matrix (Exp 1, Track A).\
        Create line charts for the performance decay curves (Exp 1, Track B and A).\
        Create bar charts comparing the "Mixed Model" vs. "Per-Level" performance (Exp 2).\
        Create grouped bar charts comparing the strategies: "w/o pre-processing," "w/ pre-processing," and "train-on-degraded" (Exp 3).\
        Create a final overlay line chart to directly compare the performance decay of the Specialist (Track A) vs. the Generalist (Track B) models.
  - [x] Draw Conclusions:\
        This phase of the experiment yielded several key conclusions regarding the model's behavior under different training strategies when faced with visual degradation.\
        Inherent Robustness & Failure Point: A specialist model trained exclusively on clear data (Strategy 1) demonstrated high inherent robustness to moderate levels of degradation. However, its performance collapsed under severe degradation (mAP50 dropped from 0.951 to 0.412), establishing a clear breaking point.\
        Efficacy of Targeted Adaptation: Fine-tuning the baseline model on a specific degradation level (Strategy 2) proved to be an exceptionally effective "repair" mechanism. This approach was able to restore performance to near-perfect levels (~0.995 mAP50) for each specific condition, making it the optimal strategy for predictable environments.\
        Generalization vs. Specialization Trade-off:\
         A single model fine-tuned on a mixed dataset of all conditions (Strategy 3) became a highly resilient "all-rounder," achieving peak performance (~0.995 mAP50) across all tested degraded conditions.\
        However, this broad robustness came at a significant cost: the model's performance on clear, non-degraded data dropped substantially (mAP50 from 0.983 down to 0.513). This highlights a classic trade-off where the model "forgets" how to specialize on ideal data in order to generalize across challenging conditions.\
        Overarching Finding: The experiments quantitatively demonstrate that for deployment in real-world conditions, there is a critical choice between training specialized models for known environments versus training a single, more versatile model that sacrifices peak performance in ideal conditions for high reliability across a wide range of degraded environments.
        
  - [ ] Finalize Report/Presentation:\
        Use the generated assets and conclusions to compile the final project report and/or presentation slides.


        
