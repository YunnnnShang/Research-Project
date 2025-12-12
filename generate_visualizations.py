#!/usr/bin/env python3
"""
Comprehensive Visualization Script for Camera Degradation Research Project
Generates all visualization charts for pre-presentation
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns
import pandas as pd
import numpy as np
import os

# Set style for publication-quality figures
sns.set_style("whitegrid")
plt.rcParams['figure.dpi'] = 300
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['font.size'] = 10
plt.rcParams['font.family'] = 'sans-serif'

# Create output directory
output_dir = "visualization_results"
os.makedirs(output_dir, exist_ok=True)

# Color palette
COLORS = {
    'primary': '#2E86AB',
    'secondary': '#A23B72',
    'tertiary': '#F18F01',
    'quaternary': '#C73E1D',
    'success': '#06A77D',
    'warning': '#F77F00',
    'danger': '#D62828',
    'neutral': '#6C757D'
}

# ============================================================================
# 1. PHASE 1: Resource Benchmark Visualization
# ============================================================================

def generate_phase1_visualizations():
    """Generate Phase 1 baseline performance visualizations"""
    print("Generating Phase 1 visualizations...")
    
    # Load data
    data_phase1 = pd.DataFrame({
        'Model': ['YOLOv8n', 'YOLOv8s', 'YOLOv8m'],
        'Avg_FPS': [2.64, 0.99, 0.41],
        'Avg_CPU_Usage': [49.83, 43.79, 39.32],
        'Avg_RAM_Usage': [21.34, 21.93, 23.30]
    })
    
    # Figure 1.1: FPS Comparison
    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(data_phase1['Model'], data_phase1['Avg_FPS'], 
                   color=[COLORS['primary'], COLORS['secondary'], COLORS['tertiary']],
                   edgecolor='black', linewidth=1.2)
    ax.set_ylabel('Average FPS', fontsize=12, fontweight='bold')
    ax.set_xlabel('Model Size', fontsize=12, fontweight='bold')
    ax.set_title('Phase 1: Model Performance - FPS Comparison\n(Raspberry Pi 5, CPU-only)', 
                 fontsize=14, fontweight='bold', pad=20)
    ax.grid(axis='y', alpha=0.3)
    
    # Add value labels on bars
    for bar in bars:
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.2f}',
                ha='center', va='bottom', fontweight='bold', fontsize=11)
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/01_Phase1_FPS_Comparison.png", bbox_inches='tight')
    plt.close()
    
    # Figure 1.2: Resource Usage Comparison
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    # CPU Usage
    bars1 = ax1.bar(data_phase1['Model'], data_phase1['Avg_CPU_Usage'], 
                    color=[COLORS['warning'], COLORS['warning'], COLORS['warning']],
                    edgecolor='black', linewidth=1.2, alpha=0.7)
    ax1.set_ylabel('CPU Usage (%)', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Model Size', fontsize=12, fontweight='bold')
    ax1.set_title('CPU Utilization', fontsize=13, fontweight='bold')
    ax1.grid(axis='y', alpha=0.3)
    ax1.set_ylim([0, 60])
    
    for bar in bars1:
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.1f}%',
                ha='center', va='bottom', fontweight='bold', fontsize=10)
    
    # RAM Usage
    bars2 = ax2.bar(data_phase1['Model'], data_phase1['Avg_RAM_Usage'], 
                    color=[COLORS['success'], COLORS['success'], COLORS['success']],
                    edgecolor='black', linewidth=1.2, alpha=0.7)
    ax2.set_ylabel('RAM Usage (%)', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Model Size', fontsize=12, fontweight='bold')
    ax2.set_title('RAM Utilization', fontsize=13, fontweight='bold')
    ax2.grid(axis='y', alpha=0.3)
    ax2.set_ylim([0, 30])
    
    for bar in bars2:
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.1f}%',
                ha='center', va='bottom', fontweight='bold', fontsize=10)
    
    fig.suptitle('Phase 1: Resource Utilization Comparison', 
                 fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()
    plt.savefig(f"{output_dir}/02_Phase1_Resource_Utilization.png", bbox_inches='tight')
    plt.close()
    
    # Figure 1.3: Multi-metric Radar Chart
    categories = ['FPS\n(normalized)', 'CPU Efficiency\n(100-usage)', 'RAM Efficiency\n(100-usage)']
    
    fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
    
    angles = np.linspace(0, 2 * np.pi, len(categories), endpoint=False).tolist()
    angles += angles[:1]
    
    # Normalize data for radar chart
    for idx, model in enumerate(data_phase1['Model']):
        fps_norm = data_phase1.loc[idx, 'Avg_FPS'] / data_phase1['Avg_FPS'].max() * 100
        cpu_eff = 100 - data_phase1.loc[idx, 'Avg_CPU_Usage']
        ram_eff = 100 - data_phase1.loc[idx, 'Avg_RAM_Usage']
        
        values = [fps_norm, cpu_eff, ram_eff]
        values += values[:1]
        
        ax.plot(angles, values, 'o-', linewidth=2, 
                label=model, color=[COLORS['primary'], COLORS['secondary'], COLORS['tertiary']][idx])
        ax.fill(angles, values, alpha=0.15)
    
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(categories, fontsize=11, fontweight='bold')
    ax.set_ylim(0, 100)
    ax.set_title('Phase 1: Multi-Metric Performance Comparison\n(Higher is Better)', 
                 fontsize=14, fontweight='bold', pad=30)
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1), fontsize=11)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/03_Phase1_Radar_Chart.png", bbox_inches='tight')
    plt.close()

# ============================================================================
# 2. PHASE 4: Generalist Model Performance
# ============================================================================

def generate_phase4_visualizations():
    """Generate Phase 4 generalist model performance visualizations"""
    print("Generating Phase 4 visualizations...")
    
    data_generalist = pd.DataFrame({
        'Degradation_Level': ['Level 0\n(Clear)', 'Level 1\n(Mild Haze)', 'Level 2\n(Moderate Haze)'],
        'Level_Num': [0, 1, 2],
        'mAP50': [0.841, 0.685, 0.074],
        'Physical_Condition': ['Clear / Control', 'Haze Level 1', 'Haze Level 2']
    })
    
    # Figure 4.1: Performance Degradation Curve
    fig, ax = plt.subplots(figsize=(10, 6))
    
    ax.plot(data_generalist['Level_Num'], data_generalist['mAP50'], 
            marker='o', linewidth=3, markersize=12, 
            color=COLORS['danger'], label='Generalist Model (YOLOv8n)')
    
    # Highlight critical drop
    ax.axhspan(0, 0.3, alpha=0.1, color='red', label='Failure Zone')
    ax.axhspan(0.3, 0.7, alpha=0.1, color='orange', label='Degraded Performance')
    ax.axhspan(0.7, 1.0, alpha=0.1, color='green', label='Good Performance')
    
    ax.set_xlabel('Degradation Level', fontsize=12, fontweight='bold')
    ax.set_ylabel('mAP@0.5', fontsize=12, fontweight='bold')
    ax.set_title('Phase 4: Generalist Model Performance Under Degradation\n(Pre-trained YOLOv8n on Bottle Detection)', 
                 fontsize=14, fontweight='bold', pad=20)
    ax.set_xticks(data_generalist['Level_Num'])
    ax.set_xticklabels(data_generalist['Degradation_Level'], fontsize=10)
    ax.set_ylim([0, 1])
    ax.grid(True, alpha=0.3)
    
    # Add value annotations
    for i, row in data_generalist.iterrows():
        ax.annotate(f'{row["mAP50"]:.3f}', 
                   xy=(row['Level_Num'], row['mAP50']),
                   xytext=(10, 10), textcoords='offset points',
                   fontsize=11, fontweight='bold',
                   bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7))
    
    # Add performance drop annotation
    ax.annotate('', xy=(1, 0.685), xytext=(0, 0.841),
               arrowprops=dict(arrowstyle='->', lw=2, color='red', linestyle='--'))
    ax.text(0.5, 0.76, '-18.5%', fontsize=11, fontweight='bold', 
            color='red', ha='center')
    
    ax.annotate('', xy=(2, 0.074), xytext=(1, 0.685),
               arrowprops=dict(arrowstyle='->', lw=2, color='darkred', linestyle='--'))
    ax.text(1.5, 0.38, '-89.2%\nCRASH', fontsize=11, fontweight='bold', 
            color='darkred', ha='center')
    
    ax.legend(loc='upper right', fontsize=10)
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/04_Phase4_Generalist_Performance_Curve.png", bbox_inches='tight')
    plt.close()
    
    # Figure 4.2: Bar chart with confidence indicator
    fig, ax = plt.subplots(figsize=(10, 6))
    
    colors_bar = ['green', 'orange', 'red']
    bars = ax.bar(data_generalist['Degradation_Level'], data_generalist['mAP50'],
                   color=colors_bar, edgecolor='black', linewidth=1.5, alpha=0.8)
    
    ax.set_ylabel('mAP@0.5', fontsize=12, fontweight='bold')
    ax.set_xlabel('Degradation Level', fontsize=12, fontweight='bold')
    ax.set_title('Phase 4: Generalist Model - Performance Drop Analysis', 
                 fontsize=14, fontweight='bold', pad=20)
    ax.set_ylim([0, 1])
    ax.axhline(y=0.5, color='gray', linestyle='--', linewidth=1, alpha=0.5, label='Usability Threshold')
    ax.grid(axis='y', alpha=0.3)
    
    for i, bar in enumerate(bars):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 0.03,
                f'{height:.3f}\n({data_generalist.loc[i, "Physical_Condition"]})',
                ha='center', va='bottom', fontweight='bold', fontsize=10)
    
    ax.legend(fontsize=10)
    plt.tight_layout()
    plt.savefig(f"{output_dir}/05_Phase4_Generalist_Bar_Chart.png", bbox_inches='tight')
    plt.close()

# ============================================================================
# 3. PHASE 5: Specialist Model Performance
# ============================================================================

def generate_phase5_visualizations():
    """Generate Phase 5 specialist model performance visualizations"""
    print("Generating Phase 5 visualizations...")
    
    # Data for all experiments
    levels = ['Level 0\n(Clear)', 'Level 1\n(Moderate)', 'Level 2\n(Heavy)', 'Level 3\n(Severe)']
    level_nums = [0, 1, 2, 3]
    
    direct_eval = [0.983, 0.995, 0.951, 0.412]
    one_to_one = [0.983, 0.995, 0.995, 0.995]  # After fine-tuning
    mixed_data = [0.513, 0.995, 0.995, 0.995]
    generalist = [0.841, 0.685, 0.074, None]  # For comparison
    
    # Figure 5.1: Complete Performance Comparison
    fig, ax = plt.subplots(figsize=(14, 7))
    
    width = 0.2
    x = np.arange(len(levels))
    
    bars1 = ax.bar(x - width*1.5, direct_eval, width, 
                   label='Specialist - Direct Eval', 
                   color=COLORS['primary'], edgecolor='black', linewidth=1)
    bars2 = ax.bar(x - width*0.5, one_to_one, width, 
                   label='Specialist - One-to-One Fine-tune', 
                   color=COLORS['success'], edgecolor='black', linewidth=1)
    bars3 = ax.bar(x + width*0.5, mixed_data, width, 
                   label='Specialist - Mixed-Data Fine-tune', 
                   color=COLORS['tertiary'], edgecolor='black', linewidth=1)
    
    # Add generalist for L0-L2 only
    generalist_vals = [0.841, 0.685, 0.074, 0]
    bars4 = ax.bar(x + width*1.5, generalist_vals, width, 
                   label='Generalist (Reference)', 
                   color=COLORS['danger'], edgecolor='black', linewidth=1, alpha=0.5)
    
    ax.set_ylabel('mAP@0.5', fontsize=13, fontweight='bold')
    ax.set_xlabel('Degradation Level', fontsize=13, fontweight='bold')
    ax.set_title('Phase 5: Complete Model Performance Comparison Across All Strategies\n(Specialist Model Performance Under Different Training Approaches)', 
                 fontsize=14, fontweight='bold', pad=20)
    ax.set_xticks(x)
    ax.set_xticklabels(levels, fontsize=11)
    ax.set_ylim([0, 1.05])
    ax.axhline(y=0.5, color='red', linestyle='--', linewidth=1.5, alpha=0.5, label='Usability Threshold')
    ax.legend(loc='lower left', fontsize=11, framealpha=0.9)
    ax.grid(axis='y', alpha=0.3)
    
    # Add value labels on bars
    for bars in [bars1, bars2, bars3]:
        for bar in bars:
            height = bar.get_height()
            if height > 0.01:
                ax.text(bar.get_x() + bar.get_width()/2., height + 0.02,
                       f'{height:.3f}',
                       ha='center', va='bottom', fontsize=8, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/06_Phase5_Complete_Comparison.png", bbox_inches='tight')
    plt.close()
    
    # Figure 5.2: Performance Curves
    fig, ax = plt.subplots(figsize=(12, 7))
    
    ax.plot(level_nums, direct_eval, marker='o', linewidth=3, markersize=10,
            label='Strategy 1: Direct Evaluation', color=COLORS['primary'])
    ax.plot(level_nums, one_to_one, marker='s', linewidth=3, markersize=10,
            label='Strategy 2: One-to-One Fine-tuning', color=COLORS['success'])
    ax.plot(level_nums, mixed_data, marker='^', linewidth=3, markersize=10,
            label='Strategy 3: Mixed-Data Fine-tuning', color=COLORS['tertiary'])
    ax.plot([0, 1, 2], generalist[:3], marker='x', linewidth=2, markersize=12,
            label='Generalist (Reference)', color=COLORS['danger'], linestyle='--', alpha=0.7)
    
    ax.set_xlabel('Degradation Level', fontsize=13, fontweight='bold')
    ax.set_ylabel('mAP@0.5', fontsize=13, fontweight='bold')
    ax.set_title('Phase 5: Specialist Model Performance Curves\n(Training Strategy Comparison)', 
                 fontsize=14, fontweight='bold', pad=20)
    ax.set_xticks(level_nums)
    ax.set_xticklabels(levels, fontsize=11)
    ax.set_ylim([0, 1.05])
    ax.axhline(y=0.5, color='red', linestyle='--', linewidth=1, alpha=0.5)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower left', fontsize=11, framealpha=0.9)
    
    # Highlight the collapse point
    ax.annotate('Performance\nCollapse Point', 
               xy=(3, 0.412), xytext=(2.5, 0.2),
               arrowprops=dict(arrowstyle='->', lw=2, color='red'),
               fontsize=11, fontweight='bold', color='red',
               bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7))
    
    # Highlight the recovery
    ax.annotate('Fine-tuning\nRecovery', 
               xy=(3, 0.995), xytext=(3.2, 0.85),
               arrowprops=dict(arrowstyle='->', lw=2, color='green'),
               fontsize=11, fontweight='bold', color='green',
               bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgreen', alpha=0.7))
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/07_Phase5_Performance_Curves.png", bbox_inches='tight')
    plt.close()
    
    # Figure 5.3: Fine-tuning Effectiveness
    fig, ax = plt.subplots(figsize=(10, 6))
    
    before_finetune = [0.995, 0.951, 0.412]
    after_finetune = [0.995, 0.995, 0.995]
    x_pos = [1, 2, 3]
    labels = ['Level 1\n(Moderate)', 'Level 2\n(Heavy)', 'Level 3\n(Severe)']
    
    width = 0.35
    bars1 = ax.bar([i - width/2 for i in x_pos], before_finetune, width,
                   label='Before Fine-tuning', color=COLORS['warning'], 
                   edgecolor='black', linewidth=1.2)
    bars2 = ax.bar([i + width/2 for i in x_pos], after_finetune, width,
                   label='After Fine-tuning', color=COLORS['success'], 
                   edgecolor='black', linewidth=1.2)
    
    ax.set_ylabel('mAP@0.5', fontsize=12, fontweight='bold')
    ax.set_xlabel('Degradation Level', fontsize=12, fontweight='bold')
    ax.set_title('Phase 5: Fine-tuning Effectiveness Analysis\n(Performance Recovery Through Targeted Fine-tuning)', 
                 fontsize=14, fontweight='bold', pad=20)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels, fontsize=11)
    ax.set_ylim([0, 1.05])
    ax.legend(fontsize=11)
    ax.grid(axis='y', alpha=0.3)
    
    # Add gain annotations
    gains = ['+0.0%', '+4.6%', '+141.5%']
    for i, gain in enumerate(gains):
        ax.text(x_pos[i], 1.0, gain, ha='center', fontsize=11, 
                fontweight='bold', color='green')
    
    # Add value labels
    for bars in [bars1, bars2]:
        for bar in bars:
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height - 0.05,
                   f'{height:.3f}',
                   ha='center', va='top', fontsize=9, fontweight='bold', color='white')
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/08_Phase5_Finetuning_Effectiveness.png", bbox_inches='tight')
    plt.close()
    
    # Figure 5.4: Trade-off Analysis (Heatmap)
    fig, ax = plt.subplots(figsize=(10, 6))
    
    data_heatmap = np.array([
        direct_eval,
        one_to_one,
        mixed_data
    ])
    
    im = ax.imshow(data_heatmap, cmap='RdYlGn', aspect='auto', vmin=0, vmax=1)
    
    ax.set_xticks(range(len(levels)))
    ax.set_xticklabels(levels, fontsize=11)
    ax.set_yticks(range(3))
    ax.set_yticklabels(['Strategy 1:\nDirect Eval', 
                        'Strategy 2:\nOne-to-One', 
                        'Strategy 3:\nMixed-Data'], fontsize=11)
    
    # Add text annotations
    for i in range(3):
        for j in range(4):
            text = ax.text(j, i, f'{data_heatmap[i, j]:.3f}',
                          ha="center", va="center", color="black", 
                          fontweight='bold', fontsize=12)
    
    ax.set_title('Phase 5: Training Strategy Performance Heatmap\n(Generalization vs. Specialization Trade-off)', 
                 fontsize=14, fontweight='bold', pad=20)
    
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('mAP@0.5', rotation=270, labelpad=20, fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/09_Phase5_Strategy_Heatmap.png", bbox_inches='tight')
    plt.close()

# ============================================================================
# 4. CROSS-PHASE COMPARISON
# ============================================================================

def generate_comprehensive_comparisons():
    """Generate comprehensive cross-phase comparison visualizations"""
    print("Generating comprehensive comparison visualizations...")
    
    # Figure C.1: Specialist vs Generalist Direct Comparison
    fig, ax = plt.subplots(figsize=(12, 7))
    
    levels = ['Level 0\n(Clear)', 'Level 1\n(Moderate)', 'Level 2\n(Heavy)', 'Level 3\n(Severe)']
    level_nums = [0, 1, 2, 3]
    
    generalist = [0.841, 0.685, 0.074, None]
    specialist = [0.983, 0.995, 0.951, 0.412]
    
    # Plot generalist (only L0-L2)
    ax.plot([0, 1, 2], generalist[:3], marker='o', linewidth=3, markersize=12,
            label='Generalist Model (Pre-trained YOLOv8n)', 
            color=COLORS['danger'], linestyle='--')
    
    # Plot specialist
    ax.plot(level_nums, specialist, marker='s', linewidth=3, markersize=12,
            label='Specialist Model (Fine-tuned on Clear Data)', 
            color=COLORS['primary'])
    
    ax.set_xlabel('Degradation Level', fontsize=13, fontweight='bold')
    ax.set_ylabel('mAP@0.5', fontsize=13, fontweight='bold')
    ax.set_title('Comprehensive Comparison: Specialist vs. Generalist Model Performance\n(Demonstrating the Value of Domain-Specific Training)', 
                 fontsize=14, fontweight='bold', pad=20)
    ax.set_xticks(level_nums)
    ax.set_xticklabels(levels, fontsize=11)
    ax.set_ylim([0, 1.05])
    
    # Add performance zones
    ax.axhspan(0, 0.3, alpha=0.1, color='red', label='Failure Zone')
    ax.axhspan(0.3, 0.7, alpha=0.1, color='orange', label='Degraded Zone')
    ax.axhspan(0.7, 1.0, alpha=0.1, color='green', label='Good Performance')
    
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower left', fontsize=11, framealpha=0.9)
    
    # Highlight key differences
    ax.annotate('Specialist\nAdvantage:\n+16.9%', 
               xy=(0, 0.983), xytext=(0.3, 0.91),
               arrowprops=dict(arrowstyle='->', lw=2, color='green'),
               fontsize=10, fontweight='bold', color='green',
               bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgreen', alpha=0.7))
    
    ax.annotate('Generalist\nCollapse', 
               xy=(2, 0.074), xytext=(1.5, 0.2),
               arrowprops=dict(arrowstyle='->', lw=2, color='red'),
               fontsize=10, fontweight='bold', color='red',
               bbox=dict(boxstyle='round,pad=0.5', facecolor='pink', alpha=0.7))
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/10_Comprehensive_Specialist_vs_Generalist.png", bbox_inches='tight')
    plt.close()
    
    # Figure C.2: Performance Summary Statistics
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
    
    # Statistical summary table
    strategies = ['Generalist\n(Pre-trained)', 'Specialist\n(Direct Eval)', 
                  'Specialist\n(One-to-One)', 'Specialist\n(Mixed-Data)']
    
    all_data = [
        [0.841, 0.685, 0.074, None],
        [0.983, 0.995, 0.951, 0.412],
        [0.983, 0.995, 0.995, 0.995],
        [0.513, 0.995, 0.995, 0.995]
    ]
    
    # 1. Mean performance across levels
    means = []
    for data in all_data:
        valid_data = [x for x in data if x is not None]
        means.append(np.mean(valid_data))
    
    bars = ax1.barh(strategies, means, 
                    color=[COLORS['danger'], COLORS['primary'], 
                           COLORS['success'], COLORS['tertiary']],
                    edgecolor='black', linewidth=1.2)
    ax1.set_xlabel('Mean mAP@0.5', fontsize=12, fontweight='bold')
    ax1.set_title('Average Performance Across All Levels', fontsize=12, fontweight='bold')
    ax1.set_xlim([0, 1])
    ax1.grid(axis='x', alpha=0.3)
    
    for i, bar in enumerate(bars):
        width = bar.get_width()
        ax1.text(width + 0.02, bar.get_y() + bar.get_height()/2.,
                f'{means[i]:.3f}',
                ha='left', va='center', fontweight='bold', fontsize=10)
    
    # 2. Performance at each level (box plot style)
    level_data = {
        'Level 0': [0.841, 0.983, 0.983, 0.513],
        'Level 1': [0.685, 0.995, 0.995, 0.995],
        'Level 2': [0.074, 0.951, 0.995, 0.995],
        'Level 3': [0.412, 0.995, 0.995]  # Removed None
    }
    
    x_pos = range(4)
    level_names = list(level_data.keys())
    
    bp = ax2.boxplot([list(v) for v in level_data.values()], 
                     positions=x_pos, widths=0.6,
                     patch_artist=True,
                     boxprops=dict(facecolor=COLORS['primary'], alpha=0.7),
                     medianprops=dict(color='red', linewidth=2),
                     whiskerprops=dict(linewidth=1.5),
                     capprops=dict(linewidth=1.5))
    
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(level_names, fontsize=11)
    ax2.set_ylabel('mAP@0.5', fontsize=12, fontweight='bold')
    ax2.set_title('Performance Distribution by Degradation Level', fontsize=12, fontweight='bold')
    ax2.set_ylim([0, 1.05])
    ax2.grid(axis='y', alpha=0.3)
    
    # 3. Robustness score (variance across levels)
    robustness = []
    for data in all_data:
        valid_data = [x for x in data if x is not None]
        robustness.append(1 - np.std(valid_data))  # Lower std = more robust
    
    bars = ax3.bar(strategies, robustness,
                   color=[COLORS['danger'], COLORS['primary'], 
                          COLORS['success'], COLORS['tertiary']],
                   edgecolor='black', linewidth=1.2, alpha=0.7)
    ax3.set_ylabel('Robustness Score\n(1 - Std Dev)', fontsize=12, fontweight='bold')
    ax3.set_title('Model Robustness Comparison', fontsize=12, fontweight='bold')
    ax3.set_ylim([0, 1])
    ax3.grid(axis='y', alpha=0.3)
    ax3.tick_params(axis='x', rotation=15)
    
    for i, bar in enumerate(bars):
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height + 0.02,
                f'{robustness[i]:.3f}',
                ha='center', va='bottom', fontweight='bold', fontsize=10)
    
    # 4. Performance gain from baseline
    baseline = [0.841, 0.685, 0.074]  # Generalist L0-L2
    specialist_gains = []
    
    for i in range(3):
        specialist_val = [0.983, 0.995, 0.951][i]
        gain = ((specialist_val - baseline[i]) / baseline[i]) * 100
        specialist_gains.append(gain)
    
    bars = ax4.bar(['Level 0', 'Level 1', 'Level 2'], specialist_gains,
                   color=[COLORS['success'] if g > 0 else COLORS['danger'] 
                          for g in specialist_gains],
                   edgecolor='black', linewidth=1.2)
    ax4.set_ylabel('Performance Gain (%)', fontsize=12, fontweight='bold')
    ax4.set_title('Specialist Model Improvement vs. Generalist', fontsize=12, fontweight='bold')
    ax4.axhline(y=0, color='black', linewidth=1)
    ax4.grid(axis='y', alpha=0.3)
    
    for i, bar in enumerate(bars):
        height = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2., height + 5 if height > 0 else height - 5,
                f'+{specialist_gains[i]:.1f}%',
                ha='center', va='bottom' if height > 0 else 'top', 
                fontweight='bold', fontsize=11, color='green')
    
    fig.suptitle('Comprehensive Performance Statistics and Analysis', 
                 fontsize=16, fontweight='bold', y=0.995)
    plt.tight_layout()
    plt.savefig(f"{output_dir}/11_Comprehensive_Statistics.png", bbox_inches='tight')
    plt.close()

# ============================================================================
# 5. SUMMARY AND KEY FINDINGS
# ============================================================================

def generate_summary_visualization():
    """Generate executive summary visualization"""
    print("Generating summary visualization...")
    
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 3, hspace=0.4, wspace=0.3)
    
    # Title
    fig.suptitle('Research Project: Camera Degradation Impact on Object Detection\nExecutive Summary Dashboard', 
                 fontsize=18, fontweight='bold', y=0.98)
    
    # Chart 1: Key Performance Metrics
    ax1 = fig.add_subplot(gs[0, :2])
    metrics = ['Best Performance\n(Clear Data)', 'Best Robustness\n(All Conditions)', 
               'Fastest Inference\n(FPS)', 'Most Cost-Effective']
    values = [0.995, 0.995, 2.64, 1.0]
    models = ['Specialist\n(One-to-One)', 'Specialist\n(Mixed)', 'YOLOv8n', 'YOLOv8n']
    
    bars = ax1.barh(metrics, values, 
                    color=[COLORS['success'], COLORS['tertiary'], COLORS['primary'], COLORS['warning']],
                    edgecolor='black', linewidth=1.2)
    ax1.set_xlabel('Score / Value', fontsize=11, fontweight='bold')
    ax1.set_title('Key Performance Highlights', fontsize=13, fontweight='bold')
    ax1.set_xlim([0, max(values) * 1.2])
    
    for i, (bar, model) in enumerate(zip(bars, models)):
        width = bar.get_width()
        ax1.text(width + 0.1, bar.get_y() + bar.get_height()/2.,
                f'{model}\n{values[i]:.2f}',
                ha='left', va='center', fontweight='bold', fontsize=9)
    
    # Chart 2: Performance degradation summary
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.axis('off')
    
    summary_text = """
    KEY FINDINGS:
    
    ✓ Specialist models outperform
      generalist by 16.9% on clear data
    
    ✓ Generalist fails at Level 2
      (mAP: 0.841 → 0.074)
    
    ✓ Specialist maintains 95%+ 
      performance through Level 2
    
    ✓ Fine-tuning recovers 141.5%
      performance at severe degradation
    
    ⚠ Mixed-data training trades
      clear performance (-47.8%)
      for robustness
    """
    
    ax2.text(0.05, 0.95, summary_text, transform=ax2.transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8),
            family='monospace')
    
    # Chart 3: Model comparison matrix
    ax3 = fig.add_subplot(gs[1, :])
    
    categories = ['Level 0', 'Level 1', 'Level 2', 'Level 3']
    generalist_data = [0.841, 0.685, 0.074, 0]
    specialist_direct = [0.983, 0.995, 0.951, 0.412]
    specialist_finetune = [0.983, 0.995, 0.995, 0.995]
    specialist_mixed = [0.513, 0.995, 0.995, 0.995]
    
    x = np.arange(len(categories))
    width = 0.2
    
    ax3.bar(x - width*1.5, generalist_data, width, label='Generalist', 
            color=COLORS['danger'], alpha=0.7)
    ax3.bar(x - width*0.5, specialist_direct, width, label='Specialist (Direct)', 
            color=COLORS['primary'], alpha=0.7)
    ax3.bar(x + width*0.5, specialist_finetune, width, label='Specialist (Fine-tuned)', 
            color=COLORS['success'], alpha=0.7)
    ax3.bar(x + width*1.5, specialist_mixed, width, label='Specialist (Mixed)', 
            color=COLORS['tertiary'], alpha=0.7)
    
    ax3.set_ylabel('mAP@0.5', fontsize=12, fontweight='bold')
    ax3.set_title('Model Performance Comparison Across All Degradation Levels', 
                  fontsize=13, fontweight='bold')
    ax3.set_xticks(x)
    ax3.set_xticklabels(categories)
    ax3.set_ylim([0, 1.05])
    ax3.legend(loc='lower left', ncol=4, fontsize=10)
    ax3.grid(axis='y', alpha=0.3)
    ax3.axhline(y=0.5, color='red', linestyle='--', alpha=0.5)
    
    # Chart 4: Recommendations
    ax4 = fig.add_subplot(gs[2, :])
    ax4.axis('off')
    
    recommendations = """
    DEPLOYMENT RECOMMENDATIONS:
    
    📊 For Known, Stable Environments:
       → Use Specialist Model with Direct Evaluation
       → Best performance on clean data (mAP@0.5 = 0.983)
       → Maintains high robustness through moderate degradation
    
    🔧 For Variable, Predictable Conditions:
       → Use Specialist Model with One-to-One Fine-tuning
       → Optimal performance across all levels (mAP@0.5 ≥ 0.995)
       → Can be fine-tuned for specific degradation types
    
    🌐 For Highly Variable, Unpredictable Environments:
       → Use Specialist Model with Mixed-Data Training
       → Excellent robustness (mAP@0.5 ≥ 0.995 for L1-L3)
       → Trade-off: 47.8% performance drop on clear data
    
    ⚡ For Resource-Constrained Deployment:
       → Use YOLOv8n (2.64 FPS on Raspberry Pi 5)
       → Acceptable for controlled environments only
       → Not suitable for degraded conditions
    """
    
    ax4.text(0.05, 0.95, recommendations, transform=ax4.transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8),
            family='monospace')
    
    plt.savefig(f"{output_dir}/12_Executive_Summary_Dashboard.png", bbox_inches='tight')
    plt.close()

# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main():
    """Main execution function"""
    print("=" * 70)
    print("Camera Degradation Research Project - Visualization Generator")
    print("=" * 70)
    print()
    
    try:
        # Generate all visualizations
        generate_phase1_visualizations()
        generate_phase4_visualizations()
        generate_phase5_visualizations()
        generate_comprehensive_comparisons()
        generate_summary_visualization()
        
        print()
        print("=" * 70)
        print("✓ All visualizations generated successfully!")
        print(f"✓ Output directory: {output_dir}/")
        print("=" * 70)
        print()
        print("Generated files:")
        print("  01_Phase1_FPS_Comparison.png")
        print("  02_Phase1_Resource_Utilization.png")
        print("  03_Phase1_Radar_Chart.png")
        print("  04_Phase4_Generalist_Performance_Curve.png")
        print("  05_Phase4_Generalist_Bar_Chart.png")
        print("  06_Phase5_Complete_Comparison.png")
        print("  07_Phase5_Performance_Curves.png")
        print("  08_Phase5_Finetuning_Effectiveness.png")
        print("  09_Phase5_Strategy_Heatmap.png")
        print("  10_Comprehensive_Specialist_vs_Generalist.png")
        print("  11_Comprehensive_Statistics.png")
        print("  12_Executive_Summary_Dashboard.png")
        print()
        print("These visualizations are ready for your pre-presentation PPT!")
        print("=" * 70)
        
    except Exception as e:
        print(f"✗ Error generating visualizations: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
