# Experimental Steps and Data Analysis Summary

## Overview
This document provides a detailed analysis of the data, model results, and analysis content for each experimental phase in the research project investigating the effects of camera degradation on image recognition algorithm performance.

---

## Phase 1: Baseline Performance Evaluation & Model Selection

### Experimental Objective
Establish accurate performance benchmarks under ideal conditions, test resource usage and performance benchmarks to determine which model size can run stably and efficiently in a constrained environment (Raspberry Pi 5).

### Experimental Data
- **Models Tested**: YOLOv8n, YOLOv8s, YOLOv8m
- **Test Environment**: Raspberry Pi 5
- **Test Conditions**: Clear, unobstructed video stream

### Available Data Files
- **File**: `Phase 1: Baseline Performance Evaluation & Model Selection/cpu_benchmark_summary.csv`
- **Data Content**:
  - Model: Model name
  - Avg_FPS: Average frames per second
  - Avg_CPU_Usage (%): Average CPU utilization
  - Avg_RAM_Usage (%): Average RAM utilization

### Experimental Results

| Model    | Avg_FPS | Avg_CPU_Usage (%) | Avg_RAM_Usage (%) |
|----------|---------|-------------------|-------------------|
| yolov8n  | 2.64    | 49.83             | 21.34             |
| yolov8s  | 0.99    | 43.79             | 21.93             |
| yolov8m  | 0.41    | 39.32             | 23.30             |

### Key Findings
1. **YOLOv8n** is the only model with minimal viability in CPU environment (~2.6 FPS)
2. Larger models have lower FPS, but resource usage differences are minimal
3. **Conclusion**: YOLOv8n was determined as the sole model for all subsequent CPU experiments

### Visualization Analysis
- **Chart Types**: 
  - "Model Size vs. FPS" bar chart
  - "Model Size vs. Resource Utilization" bar chart
- **Purpose**: Demonstrate performance vs. resource usage trade-offs for different model sizes

---

## Phase 2: Data Acquisition, Quantization & Dataset Construction

### Experimental Objective
Systematically create a well-structured and quantitatively-labeled dataset as the foundation for core experiments.

### Data Acquisition Process
1. **Scene Setup**: Experimental environment contains iRobot Create 3 (specialist target) and standard water bottle (generalist target)
2. **Condition 1 (Clear)**: Record rosbag files of robot performing 360° rotations at multiple locations → Level 0 dataset
3. **Condition 2 (Degraded)**: Using varying levels of haze or soiled glass plates, record multiple rosbag sets (3-4 distinct degradation levels)

### Data Processing Pipeline
1. **Frame Extraction**: Extract all image frames from rosbag files
2. **Quality Quantization**: Calculate BRISQUE score for every extracted frame
3. **Distribution Analysis**: Plot histogram of all BRISQUE scores to visualize distribution
4. **Level Definition**: Define formal degradation level thresholds based on histogram
   - Level 0: BRISQUE < 35 (Clear)
   - Level 1: 35-50 (Mild degradation)
   - Level 2: 50-65 (Moderate degradation)
   - Level 3: >65 (Severe degradation)

### Available Data
- **Image Datasets**: Images and annotation files classified by degradation level
  - `/data/leveled_datasets/level_0/` (Clear data)
  - `/data/leveled_datasets/level_1/` (Mild degradation)
  - `/data/leveled_datasets/level_2/` (Moderate degradation)
  - `/data/leveled_datasets/level_3/` (Severe degradation)
- **Data Split**: Each level divided into training set (80%) and validation set (20%)

### Data Characteristics
- **BRISQUE Score**: No-reference image quality assessment metric
- **Annotations**: YOLO format bounding box annotations (iRobot Create 3 detection)

---

## Phase 3: Specialist Model Training

### Experimental Objective
Train a series of specialist models for robot detection tasks.

### Training Configuration
- **Configuration File**: `Phase 3: Specialist Model Training/irobot_model.yaml`
- **Dataset Path**: H:\datasets\irobot
- **Number of Classes**: 1 (irobot)
- **Data Split**: train, val, test

### Models Trained
1. **Model_A_0 (baseline)**: Trained only on Level 0 clear data
   - Weight file: `Model_A_0_best.pt`
   
2. **Model_A_1, A_2, A_3**: Trained separately for each degradation level
   - Weight files: `Model_A_1_best.pt`, `Model_A_2_best.pt`, `Model_A_3_best.pt`
   
3. **Model_A_Mixed**: Trained on mixed data from all levels
   - Weight file: `Model_A_Mixed_best.pt`

### Available Data
- **Training Data**: Contained in `train_irobot.zip`
- **Configuration File**: `irobot_model.yaml`

### Training Parameters (Typical)
- Epochs: 50-100
- Batch size: 4-8 (depending on hardware)
- Workers: 0 (on Raspberry Pi)
- Base model: YOLOv8n

---

## Phase 4: Generalist Model Performance Evaluation

### Experimental Objective
Evaluate performance of pre-trained generalist model (yolov8n) on prepared dataset to obtain first set of core experimental results.

### Experimental Setup
- **Model**: YOLOv8n (COCO pre-trained)
- **Target**: Detect water bottle (bottle class)
- **Test Levels**: Level 0, Level 1, Level 2

### Experimental Results

| Degradation Level | Physical Condition | mAP50 Score |
|-------------------|-------------------|-------------|
| Level 0           | Clear / Control   | 0.841       |
| Level 1           | Haze Level 1      | 0.685       |
| Level 2           | Haze Level 2      | 0.074       |

### Key Findings
1. **Robust Baseline Performance**: On clear images (Level 0), yolov8n model performs well (mAP50 = 0.841)
2. **Performance Drops with Degradation**: After introducing mild blur (Level 1), model performance drops significantly (~18.5%), but remains usable
3. **Performance Crash Point**: Under moderate blur (Level 2), model performance drops dramatically, mAP50 falls to 0.074, almost completely failing
4. **Preliminary Argument**: Experimental results strongly demonstrate that reliability of standard pre-trained models decreases sharply when faced with real-world physical visual degradation, and this decrease may be nonlinear

### Visualization Analysis
- **Chart**: `map_vs_degradation.png`
- **Content**: Shows trend of mAP50 change with degradation level
- **Available Data**: `General model results.zip`

---

## Phase 5: Specialist Model Performance Under Degradation

### Experimental Objective
Fully characterize specialist model's behavior under visual degradation through three key experiments:
1. **Direct Evaluation**: Measure inherent robustness
2. **One-to-One Fine-Tuning**: Measure targeted adaptability
3. **Mixed-Data Fine-Tuning**: Create single general-purpose robust model

### Experiment 1: Direct Evaluation

**Model**: Model_R0 (trained on clear data)

#### Experimental Results

| Degradation Level | Physical Condition | mAP50 Score |
|-------------------|-------------------|-------------|
| Level 0           | Clear / Control   | 0.983       |
| Level 1           | Moderate Haze     | 0.995       |
| Level 2           | Heavy Haze        | 0.951       |
| Level 3           | Severe Haze       | 0.412       |

#### Key Findings
1. **Specialized Models Perform Excellently**: On clear images, fine-tuned specialist model (mAP50=0.983) has significantly higher baseline performance than general pre-trained model (mAP50=0.841)
2. **Specialized Models Are Extremely Robust**: Faced with moderate (Level 1) and heavy (Level 2) image degradation, specialist model's performance barely drops (0.983 → 0.995 → 0.951), showing amazing resistance
3. **Specialized Models Have Performance Inflection Point**: When image degradation reaches extremely severe (Level 3), specialist model eventually shows significant collapse, mAP50 drops sharply from 0.951 to 0.412

### Experiment 2: One-to-One Fine-Tuning Evaluation

**Method**: Starting from Model_R0, perform separate fine-tuning on each degradation level (L1-L3) to create adapted models

#### Experimental Results

| Degradation Level | mAP50 (Direct Eval Model_R0) | mAP50 (After Fine-Tuning) | Performance Gain |
|-------------------|------------------------------|---------------------------|------------------|
| Level 0           | 0.983                        | N/A (Baseline)            | N/A              |
| Level 1           | 0.995                        | 0.995                     | 0.0%             |
| Level 2           | 0.951                        | 0.995                     | +4.6%            |
| Level 3           | 0.412                        | 0.995                     | +141.5%          |

#### Key Findings
1. **Inherent Robustness**: Specialist model trained on clear data (Model_R0) demonstrated remarkable robustness against moderate and heavy degradation (Level 1 & 2), maintaining near-perfect performance
2. **Performance Breaking Point**: Under severe degradation (Level 3), Model_R0's performance collapsed from 0.951 to 0.412, proving even specialist models have clear failure thresholds
3. **Efficacy of Fine-Tuning**: Fine-tuning proved exceptionally effective. For Level 3, it "rescued" the model from near-failure state, restoring performance to near-perfect 0.995, a 141.5% relative improvement

### Experiment 3: Mixed-Data Fine-Tuning Evaluation

**Method**: Merge datasets from all levels, fine-tune Model_R0 on it to create an "all-rounder" model

#### Final Results: Master Comparison Table

| Degradation Level | mAP50 (Direct Eval) | mAP50 (One-to-One Fine-tune) | mAP50 (Mixed-Data Fine-tune) |
|-------------------|---------------------|------------------------------|------------------------------|
| Level 0           | 0.983               | N/A (Baseline)               | 0.513                        |
| Level 1           | 0.995               | 0.995                        | 0.995                        |
| Level 2           | 0.951               | 0.995                        | 0.995                        |
| Level 3           | 0.412               | 0.995                        | 0.995                        |

#### Key Findings
1. **Generalization vs. Specialization Trade-off**: 
   - Mixed-data fine-tuned model achieved peak performance (~0.995 mAP50) across all degradation conditions
   - But performance on clear, non-degraded data dropped substantially (mAP50 from 0.983 to 0.513)
2. **Model "Forgetting" Phenomenon**: Model "forgets" how to specialize on ideal data in order to generalize across challenging conditions
3. **Deployment Strategy Choice**: For real-world deployment, critical choice between training specialized models for known environments vs. training single, more versatile model that sacrifices peak performance in ideal conditions for high reliability across wide range of degraded environments

### Available Data and Files
- **Configuration Files**: `irobot_levelX.yaml` (for training/validation at different levels)
- **Result Data**:
  - `Specialist model evaluation Result.zip`
  - `finetune_on_L1.zip`, `finetune_on_L2.zip`, `finetune_on_L3.zip`
  - `Val_Mixed-Data Fine-tune_on_L0-L3.zip`
  - `irobotdetect_finetune_on_all_mixed.zip`
- **Visualizations**:
  - `model_baseline_performance.png`
  - `Specialist Model Performance Direct Evaluation vs. Fine-Tuning.png`

### Reproduction Commands
See `Reproduction Commands.md` for complete training and evaluation commands.

---

## Phase 6: Behavioral Analysis from MCAP Data

### Experimental Objective
Analyze impact of camera degradation on robot's physical driving behavior during obstacle avoidance task. This serves as "functional" or "system-level" assessment, complementing direct image quality (IQA) and object detection (YOLOv8) analyses.

### Workflow

#### Step 1: Raw Behavior Data Extraction (on Raspberry Pi)
- **Script**: `extract_behavior_to_csv.py`
- **Function**: Reads MCAP file, subscribes to `/odom` and `/cmd_vel` topics, writes complete message history for each to two separate CSV files
- **Output**:
  - `odom_data.csv`: Contains odometry data (position, velocity, etc.)
  - `cmd_vel_data.csv`: Contains command velocity (linear and angular velocity)

#### Step 2: Data Transfer
Transfer generated CSV files from Raspberry Pi to corresponding run folders on Windows PC for final analysis.

#### Step 3: Behavioral Metric Calculation & Analysis (on Windows PC)
- **Script**: `analyze_robot_behavior.py`
- **Function**: Processes CSVs from all relevant runs, calculates following key behavioral metrics:

### Behavioral Metrics

1. **Stop Percentage / Hesitation**
   - Definition: Percentage of total run time where robot's commanded velocity is effectively zero
   - Significance: Reflects degree of robot's decision-making hesitation

2. **Total Path Length**
   - Definition: Total distance in meters covered by robot, calculated from odometry data
   - Significance: Reflects robot's movement efficiency

3. **Path Smoothness (RMS Jerk)**
   - Definition: Root Mean Square of robot's jerk (rate of change of acceleration)
   - Significance: Higher value indicates more erratic, less smooth motion

### Key Findings & Interpretation

#### Complex Non-Linear Relationship
Behavioral analysis yielded complex and insightful results, revealing non-linear relationship between camera degradation and system-level performance.

1. **Inconsistent Baseline**:
   - Two control robots (r6 and r7) exhibited significantly different baseline behaviors
   - r7 showed much higher natural hesitation (11.3% stop time) than r6 (0.7%)
   - Indicates obstacle avoidance algorithm has inherent variability

2. **Non-Linear Impact of Degradation**:
   - Contrary to simple hypothesis, increased camera blur did not lead to linear increase in hesitant or erratic behavior
   - **Mildly and moderately degraded** runs (r5_8, r5_9) showed **zero hesitation**, appearing paradoxically more "confident" than control robots
   - **Most jerky and erratic** run (RMS_Jerk = 10.4) was the one with **least degradation** (r5_8)
   - **Most severely degraded** run (r5_10) was only one in experimental group to show significant hesitation (4.13%), suggesting performance cliff or change in failure mode once degradation becomes critical

3. **Core Conclusion**:
   - Impact of sensor degradation on robot's behavior is not straightforward
   - Mild degradation might "simplify" the world for algorithm by removing visual noise, leading to seemingly bolder actions
   - Highlights that evaluating system performance requires multi-faceted approach, as direct task performance (like YOLOv8) and system behavior can tell different, sometimes conflicting, stories

### Required Scripts
1. **`extract_behavior_to_csv.py`**: Run on Raspberry Pi
2. **`analyze_robot_behavior.py`**: Run on Windows PC

---

## Phase 7: Analysis, Visualization & Reporting

### Objective
Transform raw data into meaningful insights, visualizations, and conclusions.

### Completed Visualizations

1. **Performance Matrix Heatmap**
   - Purpose: Show performance across different experiments and tracks
   
2. **Performance Decay Curves**
   - Purpose: Show performance trend with degradation level
   
3. **Mixed Model vs Per-Level Model Comparison Bar Charts**
   - Purpose: Compare performance of different training strategies
   
4. **Training Strategy Comparison Grouped Bar Charts**
   - Purpose: Compare "w/o pre-processing," "w/ pre-processing," and "train-on-degraded" strategies
   
5. **Specialist vs Generalist Final Overlay Line Chart**
   - Purpose: Directly compare performance decay of Specialist (Track A) vs Generalist (Track B) models

### Core Conclusions

#### 1. Inherent Robustness & Failure Point
- Specialist model trained exclusively on clear data (Strategy 1) demonstrated high inherent robustness to moderate degradation
- But performance collapsed under severe degradation (mAP50 from 0.951 to 0.412), establishing clear breaking point

#### 2. Efficacy of Targeted Adaptation
- Fine-tuning baseline model on specific degradation level (Strategy 2) proved exceptionally effective "repair" mechanism
- This approach restored performance to near-perfect levels (~0.995 mAP50) for each specific condition
- Optimal strategy for predictable environments

#### 3. Generalization vs. Specialization Trade-off
- Single model fine-tuned on mixed dataset of all conditions (Strategy 3) became highly resilient "all-rounder"
- Achieved peak performance (~0.995 mAP50) across all tested degraded conditions
- But this broad robustness came at significant cost: model's performance on clear, non-degraded data dropped substantially (mAP50 from 0.983 to 0.513)
- Highlights classic trade-off where model "forgets" how to specialize on ideal data in order to generalize across challenging conditions

#### 4. Overarching Finding
Experiments quantitatively demonstrate that for deployment in real-world conditions, there is critical choice between training specialized models for known environments versus training single, more versatile model that sacrifices peak performance in ideal conditions for high reliability across wide range of degraded environments.

### Pending Tasks
- [ ] Finalize Report/Presentation
- [ ] Use generated assets and conclusions to compile final project report and/or presentation slides

---

## Summary: Data and Model Results Overview by Phase

### Available Data Types

1. **Performance Benchmark Data**
   - FPS, CPU usage, RAM usage (CSV format)
   
2. **Image Quality Data**
   - BRISQUE scores
   - Image datasets classified by degradation level
   
3. **Model Weight Files**
   - `.pt` files for baseline models, single-level fine-tuned models, mixed-data models
   
4. **Detection Performance Data**
   - mAP50 scores
   - Precision, recall, F1 scores
   - Confusion matrices
   
5. **Behavioral Data**
   - Odometry data (position, velocity)
   - Command velocity data
   - Stop percentage, path length, RMS Jerk metrics

### Model Results Available

1. **Resource Efficiency Analysis**
   - FPS comparison for different model sizes
   - CPU/RAM usage efficiency analysis
   
2. **Robustness Analysis**
   - Model performance curves at different degradation levels
   - Performance crash point identification
   
3. **Adaptability Analysis**
   - Performance comparison before and after fine-tuning
   - Effectiveness evaluation of different training strategies
   
4. **System-Level Performance Analysis**
   - Impact of visual degradation on robot behavior
   - Correlation between detection performance and behavioral performance

### Possible Data Analyses

1. **Quantitative Analysis**
   - Performance metric statistics (mean, standard deviation, confidence intervals)
   - Correlation analysis between degradation level and performance
   - Cost-benefit analysis of different strategies

2. **Qualitative Analysis**
   - Failure case analysis
   - Performance critical point identification
   - Deployment strategy recommendations

3. **Visualization Analysis**
   - Performance trend charts
   - Heatmaps and confusion matrices
   - Comparison bar charts and line charts

---

## Pre-Presentation Recommendations

### Presentation Focus

1. **Importance of Research Question**
   - Impact of real-world visual degradation on robotic systems

2. **Systematic Experimental Design**
   - Complete workflow from baseline evaluation to specialist model training
   - Multi-dimensional evaluation (detection performance, resource usage, behavioral performance)

3. **Key Findings**
   - Superiority and limitations of specialist models
   - Non-linear degradation impact
   - Trade-offs of different training strategies

4. **Practical Application Value**
   - Provides decision-making basis for robotic system deployment
   - Provides theoretical support for predictive maintenance

### Data Preparation Recommendations

1. Prepare main visualization charts (existing PNG files)
2. Prepare key data tables (extracted from CSV)
3. Prepare model performance comparison tables
4. Prepare behavioral analysis result summaries

### Potential Question Preparation

1. **Q**: Why choose BRISQUE instead of other IQA metrics?
   **A**: BRISQUE is a no-reference metric, doesn't require original clear images as reference

2. **Q**: Is performance drop on clear images acceptable for mixed-data model?
   **A**: Depends on application scenario; if environment varies widely, this is acceptable trade-off

3. **Q**: How to interpret non-linear results in behavioral analysis?
   **A**: Mild degradation might simplify visual information, reducing decision complexity

---

## Reference Locations

- Project README: `/README.md`
- Experimental Roadmap: `/Experimental Roadmap.md`
- Phase 1 Data: `/Phase 1: Baseline Performance Evaluation & Model Selection/`
- Phase 3 Configuration: `/Phase 3: Specialist Model Training/`
- Phase 4 Results: `/Phase 4: Generalist Model Performance Evaluation/`
- Phase 5 Results and Commands: `/Phase 5-Specialist model evaluation/`
