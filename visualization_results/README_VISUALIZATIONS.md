# 可视化结果说明 / Visualization Results Guide

## 概述 / Overview

本目录包含为Pre-presentation准备的所有可视化图表，涵盖性能指标统计、模型在不同降解级别下的性能曲线、微调前后的性能对比以及不同训练策略的效果评估。

This directory contains all visualization charts prepared for the pre-presentation, covering performance metric statistics, model performance curves at different degradation levels, performance comparison before and after fine-tuning, and evaluation of different training strategies.

---

## 生成的可视化图表 / Generated Visualizations

### Phase 1: 基线性能评估 / Baseline Performance Evaluation

#### 01_Phase1_FPS_Comparison.png
- **内容**: YOLOv8n, YOLOv8s, YOLOv8m三个模型的FPS对比
- **用途**: 展示在Raspberry Pi 5 (CPU-only)上不同模型大小的推理速度
- **关键发现**: YOLOv8n以2.64 FPS的速度是唯一可行的CPU推理模型
- **PPT建议**: 用于介绍实验环境和模型选择依据

**Content**: FPS comparison of YOLOv8n, YOLOv8s, YOLOv8m models  
**Purpose**: Show inference speed of different model sizes on Raspberry Pi 5 (CPU-only)  
**Key Finding**: YOLOv8n at 2.64 FPS is the only viable CPU inference model  
**PPT Suggestion**: Use for introducing experimental environment and model selection rationale

---

#### 02_Phase1_Resource_Utilization.png
- **内容**: CPU和RAM使用率的对比图（双子图）
- **用途**: 展示不同模型大小的资源消耗
- **关键发现**: 模型大小对资源使用影响相对较小（CPU: 39-50%, RAM: 21-23%）
- **PPT建议**: 用于说明资源约束条件

**Content**: Comparison of CPU and RAM utilization (dual subplot)  
**Purpose**: Show resource consumption of different model sizes  
**Key Finding**: Model size has relatively small impact on resource usage (CPU: 39-50%, RAM: 21-23%)  
**PPT Suggestion**: Use for explaining resource constraints

---

#### 03_Phase1_Radar_Chart.png
- **内容**: 多指标雷达图（FPS、CPU效率、RAM效率）
- **用途**: 综合展示三个模型的多维性能
- **关键发现**: YOLOv8n在FPS和效率之间取得最佳平衡
- **PPT建议**: 作为Phase 1的总结图表

**Content**: Multi-metric radar chart (FPS, CPU efficiency, RAM efficiency)  
**Purpose**: Comprehensively show multi-dimensional performance of three models  
**Key Finding**: YOLOv8n achieves best balance between FPS and efficiency  
**PPT Suggestion**: Use as summary chart for Phase 1

---

### Phase 4: 通用模型性能评估 / Generalist Model Performance

#### 04_Phase4_Generalist_Performance_Curve.png
- **内容**: 通用模型在不同降解级别下的性能曲线
- **用途**: 展示预训练YOLOv8n在视觉降解下的性能衰减
- **关键发现**: 
  - Level 0 → Level 1: -18.5% (mAP50: 0.841 → 0.685)
  - Level 1 → Level 2: -89.2% (mAP50: 0.685 → 0.074) **性能崩溃**
- **PPT建议**: 用于强调问题的严重性和研究的必要性

**Content**: Performance curve of generalist model at different degradation levels  
**Purpose**: Show performance degradation of pre-trained YOLOv8n under visual degradation  
**Key Findings**:
  - Level 0 → Level 1: -18.5% (mAP50: 0.841 → 0.685)
  - Level 1 → Level 2: -89.2% (mAP50: 0.685 → 0.074) **Performance Crash**  
**PPT Suggestion**: Use to emphasize severity of problem and necessity of research

---

#### 05_Phase4_Generalist_Bar_Chart.png
- **内容**: 通用模型性能下降分析（柱状图）
- **用途**: 更直观地展示三个降解级别的性能差异
- **关键发现**: 在Level 2，性能跌破可用性阈值（0.5）
- **PPT建议**: 搭配性能曲线图使用，增强视觉冲击力

**Content**: Generalist model performance drop analysis (bar chart)  
**Purpose**: More intuitively show performance differences across three degradation levels  
**Key Finding**: At Level 2, performance falls below usability threshold (0.5)  
**PPT Suggestion**: Use together with performance curve chart for enhanced visual impact

---

### Phase 5: 专用模型性能评估 / Specialist Model Performance

#### 06_Phase5_Complete_Comparison.png ⭐核心图表
- **内容**: 所有策略的完整性能对比（4种模型 × 4个降解级别）
- **用途**: 展示专用模型与通用模型的全面对比
- **关键发现**: 
  - 专用模型在Level 0上优于通用模型16.9% (0.983 vs 0.841)
  - 专用模型对中度降解具有强鲁棒性
  - 微调策略可在所有级别达到0.995的性能
- **PPT建议**: **必用图表**，展示核心研究贡献

**Content**: Complete performance comparison of all strategies (4 models × 4 degradation levels)  
**Purpose**: Show comprehensive comparison between specialist and generalist models  
**Key Findings**:
  - Specialist outperforms generalist by 16.9% at Level 0 (0.983 vs 0.841)
  - Specialist has strong robustness to moderate degradation
  - Fine-tuning strategy achieves 0.995 performance across all levels  
**PPT Suggestion**: **Must-use chart**, showcases core research contribution

---

#### 07_Phase5_Performance_Curves.png
- **内容**: 三种训练策略的性能曲线对比
- **用途**: 展示不同策略下的性能演变趋势
- **关键发现**: 
  - 策略1（直接评估）在Level 3崩溃
  - 策略2（一对一微调）实现完美恢复
  - 策略3（混合数据）牺牲Level 0性能换取鲁棒性
- **PPT建议**: 用于详细解释三种策略的优劣

**Content**: Performance curve comparison of three training strategies  
**Purpose**: Show performance evolution trends under different strategies  
**Key Findings**:
  - Strategy 1 (direct eval) crashes at Level 3
  - Strategy 2 (one-to-one fine-tune) achieves perfect recovery
  - Strategy 3 (mixed-data) sacrifices Level 0 performance for robustness  
**PPT Suggestion**: Use for detailed explanation of advantages/disadvantages of three strategies

---

#### 08_Phase5_Finetuning_Effectiveness.png
- **内容**: 微调前后的性能对比（针对Level 1-3）
- **用途**: 量化微调策略的有效性
- **关键发现**: 
  - Level 1: +0.0% (已经很好)
  - Level 2: +4.6% (轻微改善)
  - Level 3: +141.5% (显著恢复！)
- **PPT建议**: 用于强调微调在严重降解条件下的"救援"能力

**Content**: Performance comparison before and after fine-tuning (for Level 1-3)  
**Purpose**: Quantify effectiveness of fine-tuning strategy  
**Key Findings**:
  - Level 1: +0.0% (already good)
  - Level 2: +4.6% (slight improvement)
  - Level 3: +141.5% (significant recovery!)  
**PPT Suggestion**: Use to emphasize "rescue" capability of fine-tuning under severe degradation

---

#### 09_Phase5_Strategy_Heatmap.png
- **内容**: 训练策略性能热力图（3种策略 × 4个级别）
- **用途**: 直观展示泛化与专业化的权衡
- **关键发现**: 混合数据训练在Level 0的性能损失（0.513）非常明显
- **PPT建议**: 用于可视化"没有免费的午餐"原理

**Content**: Training strategy performance heatmap (3 strategies × 4 levels)  
**Purpose**: Intuitively show generalization vs. specialization trade-off  
**Key Finding**: Performance loss at Level 0 (0.513) with mixed-data training is very significant  
**PPT Suggestion**: Use to visualize "no free lunch" principle

---

### 综合对比 / Comprehensive Comparisons

#### 10_Comprehensive_Specialist_vs_Generalist.png ⭐核心图表
- **内容**: 专用模型与通用模型的直接对比曲线
- **用途**: 展示专用模型的核心优势
- **关键发现**: 
  - 在Level 0，专用模型优势+16.9%
  - 在Level 2，通用模型完全失效，专用模型仍保持95%+性能
- **PPT建议**: **必用图表**，作为结论部分的关键证据

**Content**: Direct comparison curve of specialist vs. generalist models  
**Purpose**: Show core advantages of specialist model  
**Key Findings**:
  - At Level 0, specialist advantage +16.9%
  - At Level 2, generalist completely fails, specialist maintains 95%+ performance  
**PPT Suggestion**: **Must-use chart**, key evidence for conclusion section

---

#### 11_Comprehensive_Statistics.png
- **内容**: 4个子图的综合统计分析
  - 左上：各策略的平均性能
  - 右上：各级别的性能分布（箱线图）
  - 左下：模型鲁棒性评分
  - 右下：专用模型相对通用模型的性能增益
- **用途**: 提供深入的统计洞察
- **PPT建议**: 作为详细分析的补充材料

**Content**: Comprehensive statistical analysis with 4 subplots
  - Top-left: Average performance of each strategy
  - Top-right: Performance distribution by level (box plot)
  - Bottom-left: Model robustness score
  - Bottom-right: Performance gain of specialist over generalist  
**Purpose**: Provide in-depth statistical insights  
**PPT Suggestion**: Use as supplementary material for detailed analysis

---

#### 12_Executive_Summary_Dashboard.png ⭐核心图表
- **内容**: 执行摘要仪表板，包含：
  - 关键性能亮点
  - 核心发现总结
  - 所有模型的完整对比矩阵
  - 部署建议
- **用途**: 一图展示整个研究的核心成果
- **PPT建议**: **必用图表**，作为开场或结束的总结页

**Content**: Executive summary dashboard, including:
  - Key performance highlights
  - Core findings summary
  - Complete comparison matrix of all models
  - Deployment recommendations  
**Purpose**: Show core achievements of entire research in one chart  
**PPT Suggestion**: **Must-use chart**, as opening or closing summary slide

---

## PPT结构建议 / PPT Structure Suggestions

### 推荐的图表使用顺序 / Recommended Chart Usage Order

#### 1. 引言 (Introduction) - 3-4 slides
- **Slide 1**: 研究背景和动机
- **Slide 2**: `01_Phase1_FPS_Comparison.png` - 硬件环境和模型选择
- **Slide 3**: `04_Phase4_Generalist_Performance_Curve.png` - 问题的严重性
- **Slide 4**: 研究目标和方法论

#### 2. 基线评估 (Baseline Evaluation) - 2-3 slides
- **Slide 5**: `02_Phase1_Resource_Utilization.png` - 资源约束分析
- **Slide 6**: `03_Phase1_Radar_Chart.png` - 模型综合性能
- **Slide 7**: Phase 1 小结

#### 3. 核心结果 (Core Results) - 5-6 slides ⭐
- **Slide 8**: `06_Phase5_Complete_Comparison.png` - 所有策略完整对比
- **Slide 9**: `07_Phase5_Performance_Curves.png` - 策略演变趋势
- **Slide 10**: `08_Phase5_Finetuning_Effectiveness.png` - 微调有效性
- **Slide 11**: `10_Comprehensive_Specialist_vs_Generalist.png` - 专用vs通用
- **Slide 12**: `09_Phase5_Strategy_Heatmap.png` - 权衡分析
- **Slide 13**: Phase 5 关键发现总结

#### 4. 深入分析 (In-depth Analysis) - 2-3 slides
- **Slide 14**: `11_Comprehensive_Statistics.png` - 统计分析
- **Slide 15**: `05_Phase4_Generalist_Bar_Chart.png` - 通用模型失效分析
- **Slide 16**: 讨论和洞察

#### 5. 结论与建议 (Conclusions & Recommendations) - 2-3 slides
- **Slide 17**: `12_Executive_Summary_Dashboard.png` - 执行摘要
- **Slide 18**: 研究贡献和局限性
- **Slide 19**: 未来工作

---

## 关键数据速查 / Key Data Quick Reference

### 性能指标 / Performance Metrics

| 模型/策略 | Level 0 | Level 1 | Level 2 | Level 3 | 平均 |
|----------|---------|---------|---------|---------|------|
| 通用模型 (Generalist) | 0.841 | 0.685 | 0.074 | - | 0.533 |
| 专用-直接评估 (Direct) | 0.983 | 0.995 | 0.951 | 0.412 | 0.835 |
| 专用-一对一微调 (Fine-tune) | 0.983 | 0.995 | 0.995 | 0.995 | 0.992 |
| 专用-混合数据 (Mixed) | 0.513 | 0.995 | 0.995 | 0.995 | 0.875 |

### 关键改进 / Key Improvements

- **专用模型优势** (Level 0): +16.9% (0.983 vs 0.841)
- **微调恢复能力** (Level 3): +141.5% (0.412 → 0.995)
- **鲁棒性成本** (Mixed-data, Level 0): -47.8% (0.983 → 0.513)

### 资源使用 / Resource Usage

- **YOLOv8n**: 2.64 FPS, 49.83% CPU, 21.34% RAM
- **YOLOv8s**: 0.99 FPS, 43.79% CPU, 21.93% RAM
- **YOLOv8m**: 0.41 FPS, 39.32% CPU, 23.30% RAM

---

## 使用建议 / Usage Tips

### 针对不同受众 / For Different Audiences

#### 1. 技术专家 (Technical Experts)
- 重点使用: 06, 07, 09, 11
- 强调: 统计显著性、方法论细节、性能曲线

#### 2. 项目管理者 (Project Managers)
- 重点使用: 10, 12, 08
- 强调: ROI、部署建议、成本效益分析

#### 3. 学术评审 (Academic Reviewers)
- 重点使用: 06, 07, 10, 11
- 强调: 实验设计、统计分析、关键洞察

#### 4. 一般听众 (General Audience)
- 重点使用: 12, 10, 04
- 强调: 简化的故事线、清晰的视觉对比

---

## 高分辨率设置 / High-Resolution Settings

所有图表均以300 DPI生成，适合：
- ✓ 打印海报
- ✓ 高清投影
- ✓ 论文出版
- ✓ PPT演示

All charts generated at 300 DPI, suitable for:
- ✓ Poster printing
- ✓ HD projection
- ✓ Paper publication
- ✓ PPT presentation

---

## 颜色方案 / Color Scheme

图表使用统一的专业配色方案：
- **主色** (Primary): 蓝色 - 专用模型直接评估
- **次色** (Secondary): 紫色 - 用于对比
- **第三色** (Tertiary): 橙色 - 混合数据训练
- **成功** (Success): 绿色 - 一对一微调
- **警告** (Warning): 黄橙色 - CPU使用
- **危险** (Danger): 红色 - 通用模型/失效区域

Charts use unified professional color scheme:
- **Primary**: Blue - Specialist model direct evaluation
- **Secondary**: Purple - For contrast
- **Tertiary**: Orange - Mixed-data training
- **Success**: Green - One-to-one fine-tuning
- **Warning**: Yellow-orange - CPU usage
- **Danger**: Red - Generalist model/failure zone

---

## 常见问题 / FAQ

**Q1: 哪个图表最重要？**  
A: `06_Phase5_Complete_Comparison.png`, `10_Comprehensive_Specialist_vs_Generalist.png`, 和 `12_Executive_Summary_Dashboard.png` 是三个核心图表。

**Q2: 如何快速展示核心贡献？**  
A: 使用 `12_Executive_Summary_Dashboard.png` 作为单页总结。

**Q3: 如何强调微调的价值？**  
A: 使用 `08_Phase5_Finetuning_Effectiveness.png`，特别关注Level 3的141.5%改进。

**Q4: 如何解释权衡？**  
A: 使用 `09_Phase5_Strategy_Heatmap.png` 或 `07_Phase5_Performance_Curves.png`。

---

**生成日期 / Generated**: 2025-12-12  
**脚本 / Script**: `generate_visualizations.py`  
**总图表数 / Total Charts**: 12  
**分辨率 / Resolution**: 300 DPI  
**格式 / Format**: PNG (RGBA)
