# /usr/bin/env python3

# File: run_cpu_benchmark.py
# Description: A script to run batch performance benchmarks for YOLOv8 models 
#              on a Raspberry Pi 5's CPU, using the Picamera2 library for
#              stable, native camera access.

import time
import psutil
import pandas as pd
from ultralytics import YOLO
from picamera2 import Picamera2

def run_single_model_benchmark(model_path: str, num_frames: int = 300) -> dict | None:
    """
    Runs a benchmark for a single YOLOv8 model and returns summary statistics.

    Args:
        model_path (str): Path to the .pt model file.
        num_frames (int): The total number of frames to use for the test.

    Returns:
        dict | None: A dictionary containing the summary statistics, or None if the test fails.
    """
    print(f"\n--- Benchmarking Model: {model_path} ---")

    # 1. Initialize Picamera2
    try:
        picam2 = Picamera2()
        # Request a 3-channel BGR format compatible with YOLOv8/OpenCV.
        config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
        picam2.configure(config)
        picam2.start()
        print("Picamera2 camera initialized successfully.")
        # Allow the camera sensor to warm up and stabilize.
        time.sleep(2)
    except Exception as e:
        print(f"Error: Failed to initialize Picamera2. Error: {e}")
        return None

    # 2. Load the YOLO model
    try:
        model = YOLO(model_path)
        print(f"YOLO model '{model_path}' loaded successfully.")
    except Exception as e:
        print(f"Error: Failed to load YOLO model '{model_path}': {e}")
        picam2.stop()
        return None

    # 3. Prepare for data logging
    metrics = []
    
    print(f"Starting benchmark for {num_frames} frames...")
    
    # 4. Run the main benchmark loop
    for i in range(num_frames):
        frame_start_time = time.time()
        
        # Capture a 3-channel frame from Picamera2 (NumPy array).
        frame = picam2.capture_array()
        
        # Run inference on the 3-channel frame.
        model(frame, verbose=False)
        
        frame_end_time = time.time()

        # Calculate metrics for the current frame.
        elapsed_time = frame_end_time - frame_start_time
        fps = 1.0 / elapsed_time if elapsed_time > 0 else 0
        cpu_usage = psutil.cpu_percent()
        ram_usage = psutil.virtual_memory().percent
        
        metrics.append({'fps': fps, 'cpu_percent': cpu_usage, 'ram_percent': ram_usage})
        
        if (i + 1) % 100 == 0:
            print(f"  Processed {i + 1}/{num_frames} frames...")

    # 5. Release resources
    picam2.stop()
    print("Camera stopped.")

    # 6. Calculate summary statistics
    df = pd.DataFrame(metrics)
    # Ignore the first 10 frames to get more stable results, avoiding initial jitters.
    stable_df = df.iloc[10:]
    
    summary = {
        "Model": model_path.replace('.pt', ''),
        "Avg_FPS": stable_df['fps'].mean(),
        "Avg_CPU_Usage_Percent": stable_df['cpu_percent'].mean(),
        "Avg_RAM_Usage_Percent": stable_df['ram_percent'].mean()
    }
    
    # Save detailed per-frame data for each model for traceability.
    detailed_csv_path = f"benchmark_details_{summary['Model']}.csv"
    df.to_csv(detailed_csv_path, index=False)
    print(f"Detailed per-frame data saved to: {detailed_csv_path}")

    return summary

def main():
    """
    Main function to orchestrate the batch testing of all specified models.
    """
    # --- Define all models to be tested here ---
    models_to_test = ['yolov8n.pt', 'yolov8s.pt', 'yolov8m.pt']
    
    all_results = []

    for model_name in models_to_test:
        # Run the benchmark for a single model.
        result = run_single_model_benchmark(model_name)
        if result:
            all_results.append(result)
        else:
            print(f"Benchmark for model {model_name} failed. Skipping.")
    
    if not all_results:
        print("All model benchmarks failed. Cannot generate summary report.")
        return

    # --- Generate and output the final summary report ---
    summary_df = pd.DataFrame(all_results)
    
    print("\n\n" + "="*65)
    print("      RASPBERRY PI 5 CPU BENCHMARK - FINAL SUMMARY")
    print("="*65)
    print(summary_df.to_string(index=False, float_format="%.2f"))
    print("="*65 + "\n")

    # --- Save the summary report to a single CSV file ---
    summary_csv_path = "cpu_benchmark_summary.csv"
    summary_df.to_csv(summary_csv_path, index=False, float_format="%.2f")
    print(f"Final summary report saved to: {summary_csv_path}")


if __name__ == '__main__':
    main()
