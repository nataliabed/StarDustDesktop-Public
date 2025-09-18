"""
Configuration file for Master Motion Capture System
All paths are relative to this script's location for portability
"""

import os

# Get the directory where this config file is located
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))  # Go up to Unity project root (Assets/Scripts -> Assets -> Unity project root)

# --- Input Files ---
# Raw data files that will be processed by the motion capture pipeline

# Video file path (Python_Data directory in Unity project)
VIDEO_PATH = os.path.join(SCRIPT_DIR, "Python_Data", "video.mov")

# Depth data binary file path (Python_Data directory in Unity project)
DEPTH_JSON_PATH = os.path.join(SCRIPT_DIR, "Python_Data", "depth.bin")

# --- Output Destination ---
# Directory where all processed files will be saved for Unity to access

# Output directory for all generated files (StreamingAssets directory in Unity project)
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "Assets", "StreamingAssets")

# --- Camera Calibration Parameters ---
# Intrinsic camera parameters obtained from checkerboard calibration

CAMERA_FX = 1478.49094  # Focal length in x (pixels)
CAMERA_FY = 1477.42987  # Focal length in y (pixels)
CAMERA_CX = 952.359616  # Principal point x (center of image)
CAMERA_CY = 717.874269  # Principal point y (center of image)

USDZ_MODEL_PATH = os.path.join(SCRIPT_DIR, "Python_Data", "room_model.usdz")
CAMERA_POSE_PATH = os.path.join(SCRIPT_DIR, "Python_Data", "camera_pose.json")