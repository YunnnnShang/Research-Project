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
**Status:** ▶️ In Progress

**Description:** Before investigating performance degradation, it is crucial to establish a precise performance baseline under ideal conditions. This phase addresses the resource usage and performance benchmarking outlined in the initial roadmap.

**Checklist:**
  - [ ] Resource and Performance Benchmarking (on Raspberry Pi 5):\
        Objective: Test yolov8n, yolov8s, and yolov8m models on a clear, unobstructed video stream.\
        Metrics to Measure:\
            1. FPS: Real-time inference speed for each model size.\
            2. Resource Utilization: CPU and RAM usage during inference, measured using psutil.\
        Deliverable: Generate plots visualizing "Model Size vs. FPS" and "Model Size vs. Resource Utilization".
  - [ ] Model Architecture Selection:\
        Objective: Based on the benchmark results, select one or two optimal model sizes for all subsequent degradation experiments.\
        Criteria: Balance inference speed, performance, and resource constraints. For example, select `yolov8n.pt` as the "high-efficiency" model and `yolov8s.pt `as the "balanced-performance" model. This decision will be consistently applied throughout the research.

### Phase 2: Data Acquisition, Quantization & Dataset Construction
**Status:** `☐ To-Do`

**Description:** Systematically create a well-structured and quantitatively-labeled dataset, which will be the foundation for the core experiments.

**Checklist:**
  - [ ] Systematic Data Collection:\
        Scene Setup: Ensure the experimental environment contains both the **iRobot Create 3** (the specialist target) and a standard water bottle (the generalist target).\
        Condition 1 (Clear): Record `rosbag` files of the robot performing 360° rotations at multiple locations. This will serve as the Level 0 (clear) dataset.\
        Condition 2 (Degraded): Using varying levels of haze or soiled glass plates, record multiple `rosbag` sets (e.g., 3-4 distinct levels of degradation) under the same conditions as above.
  - [ ] Data Processing and Quantization:\
        Execute scripts to extract all image frames from the `rosbag` files.\
        Calculate the BRISQUE score for every extracted frame.\
        Plot a histogram of all BRISQUE scores to visualize the distribution.
  - [ ] Define Degradation Levels:\
        Based on the histogram, define formal thresholds for `Level 0, 1, 2, 3....` For example: `Level 0: BRISQUE < 35`, `Level 1: 35-50`, etc.
  - [ ] Construct Leveled Datasets:\
        Write a script to automatically sort all images and their corresponding robot annotation files into leveled directories (`/data/leveled_datasets/level_0`, `/level_1`, etc.) based on their BRISQUE scores.\
        For each level, split the data into `train` (80%) and `val` (20%) subsets.

### Phase 3: Specialist Model Training
**Status:** `☐ To-Do`

**Description:** Corresponds to the "Transfer Learning" milestone. This phase involves training a series of specialist models for the robot detection task.

**Checklist:**
  - [ ] Train Baseline Model: Train `Model_A_0` using only the clean `Level 0 `training data. Save the best weights as `Model_A_0_best.pt`.
  - [ ] Train Per-Level Models: For each degradation level (`Level 1, 2, 3...`), train a separate specialist model using its corresponding training data. Save the weights as `Model_A_1_best.pt`, `Model_A_2_best.pt`, etc.
  - [ ] Train Mixed-Data Model: Combine the training sets from all levels to train a single, robust `Model_A_Mixed_best.pt`.
        
### Phase 4: Systematic Degradation Experiment Execution
**Status:** `☐ To-Do`

**Description:** The core execution phase of the research, where all raw performance data is generated.

**Checklist:**
  - [ ] Execute Experiment 1 (Cross-Level Validation):\
        Track A (Specialist Model): Evaluate every trained model (`Model_A_i`) on every level's validation set (`Level_j`). Log the results in a performance matrix.
        Track B (Generalist Model): Evaluate the single pre-trained model (`Model_B`) on every level's validation set (for "bottle" detection). Log the results to plot a performance decay curve.
  - [ ] Execute Experiment 2 (Mixed-Data Robustness):\
        Track A (Specialist Model): Evaluate the `Model_A_Mixed` on every level's validation set and log the results.
  - [ ] Execute Experiment 3 (Efficacy of Dehazing Pre-processing):\
        Write and run a dehazing script on all degraded validation sets to create "restored" versions.\
        Evaluate both `Model_A_0` and `Model_B` on these restored datasets and log the results.
### Phase 5: Analysis, Visualization & Reporting
**Status:** `☐ To-Do`

**Description:** Transform the raw data into meaningful insights, visualizations, and conclusions.

**Checklist:**
  - [ ] Generate Key Visualizations:\
        Create a heatmap for the performance matrix (Exp 1, Track A).\
        Create line charts for the performance decay curves (Exp 1, Track B and A).\
        Create bar charts comparing the "Mixed Model" vs. "Per-Level" performance (Exp 2).\
        Create grouped bar charts comparing the strategies: "w/o pre-processing," "w/ pre-processing," and "train-on-degraded" (Exp 3).\
        Create a final overlay line chart to directly compare the performance decay of the Specialist (Track A) vs. the Generalist (Track B) models.
  - [ ] Draw Conclusions:\
        Based on the visualizations, formulate answers to the core research questions. (e.g., "The fine-tuned specialist model shows greater resilience to minor degradation but suffers a steeper performance decline under heavy degradation compared to the generalist model.").
  - [ ] Finalize Report/Presentation:\
        Use the generated assets and conclusions to compile the final project report and/or presentation slides.


        
