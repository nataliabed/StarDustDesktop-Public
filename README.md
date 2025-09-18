# StarDust Unity Motion Capture System

A Unity-based visualization system that processes video and depth data to create ghostly 3D animations of human movement within reconstructed room environments.

## System Requirements

### Python Environment
- **Python 3.12.2** (tested version)

### System Dependencies

#### Blender (Required for USDZ conversion unless doing manual conversion in a different software of your preference)
- **Version**: Blender 3.0 or higher
- **Installation**:
  - **macOS**: Download from [blender.org](https://www.blender.org/download/)

#### FFmpeg (Required for video processing)
- **Installation**:
  - **macOS**: `brew install ffmpeg`

### Unity Requirements
- Unity 2022.3 LTS or newer
- **Required Unity Packages** (install via Package Manager):
  - Universal Render Pipeline (URP) 14.0.11
  - Visual Effect Graph 14.0.11
  - Shader Graph 14.0.11
  - Newtonsoft JSON 3.2.1
  - Timeline 1.7.6

### Data Collection Requirements
- **For creating your own data**: iPhone 13 Pro or newer with LiDAR sensor
- **For using example data**: No special hardware required (example files included)

## Installation

### 1. Python Environment Setup

#### Using conda
```bash
# Create a new conda environment with tested Python version
conda create -n stardust python=3.12.2
conda activate stardust

# Install Python dependencies
pip install -r requirements.txt
```

#### Using pip
```bash
# Install Python 3.12.2
# Install dependencies
pip install -r requirements.txt
```

### 2. Unity Project Setup

1. Open the project in Unity 2022.3 LTS or newer
2. **Install required Unity packages**:
   - Open Window → Package Manager
   - Install the following packages:
     - Universal RP (14.0.11)
     - Visual Effect Graph (14.0.11) 
     - Shader Graph (14.0.11)
     - Newtonsoft Json (3.2.1)
     - Timeline (1.7.6)
3. The project uses relative paths and should work on any machine
4. Configure Python path in the MemoryPlayer component

### 3. Data Setup

The system needs 4 data files in `Assets/Scripts/Python_Data/`:
- `video.mov` - Source video file
- `depth.bin` - Depth data (binary format)
- `room_model.usdz` - 3D room model
- `camera_pose.json` - Camera calibration data

**To get these files:**

1. **Extract from example zip files**: The project includes 2 example datasets (kitchen and living room) as zip files. Extract any example's 4 files to the root Python_Data folder:
   ```bash
   # For kitchen example
   unzip Assets/Scripts/Python_Data/kitchen_example.zip -d Assets/Scripts/Python_Data/kitchen_example/
   cp Assets/Scripts/Python_Data/kitchen_example/* Assets/Scripts/Python_Data/
   
   # For living room example
   unzip Assets/Scripts/Python_Data/living_example.zip -d Assets/Scripts/Python_Data/living_example/
   cp Assets/Scripts/Python_Data/living_example/* Assets/Scripts/Python_Data/
   ```

2. **Use your own data**: If you have StarDust Mobile data (requires iPhone 13 Pro+ with LiDAR), place your 4 files directly in `Assets/Scripts/Python_Data/`.

## Usage

### Automated Processing
1. Open the Unity project
2. Play the scene - the system will automatically:
   - Check for existing processed files
   - Run Python motion capture if needed
   - Load and display the visualization

### Manual Processing
```bash
# Run motion capture pipeline
python Assets/Scripts/master_motion_capture.py mocap --headless

# Run with visualizations (for debugging)
python Assets/Scripts/master_motion_capture.py mocap
```

## Configuration

Edit `Assets/Scripts/config.py` to modify:
- File paths (automatically relative to script location)
- Camera calibration parameters
- Output directory

## Troubleshooting

### Python Processing Issues
- **Problem**: Python motion capture isn't working or fails to process
- **Solution**: Try the pre-processed example in `Assets/StreamingAssets/processed_example.zip`
  - Extract it to see the visualization without running Python processing
  - This contains pre-processed room_example data for immediate visualization

### Blender Not Found
- **Problem**: "Blender not found" error during USDZ conversion
- **Solution**: Install Blender 3.0+ and ensure it's in your PATH

### FFmpeg Missing
- **Problem**: Video processing fails
- **Solution**: Install FFmpeg system-wide

### Python Path Issues
- **Problem**: Unity can't find Python
- **Solution**: Set the Python path in the MemoryPlayer component

## File Structure

```
Assets/Scripts/
├── config.py                    # Configuration settings
├── master_motion_capture.py     # Python processing pipeline
├── MemoryPlayer.cs              # Unity controller
└── Python_Data/                 # Input data directory
    ├── kitchen_example.zip      # Kitchen scene data (extract to use)
    ├── living_example.zip       # Living room scene data (extract to use)
    ├── video.mov                # Active data files (copied from examples)
    ├── depth.bin
    ├── room_model.usdz
    └── camera_pose.json

Assets/StreamingAssets/           # Generated output files
├── processed_example.zip        # Pre-processed room example (extract to visualize)
└── [generated files]            # Created by Python processing:
    ├── room_model.obj
    ├── room_model.mtl
    ├── camera_pose.json
    └── master_tracking_results.json
```

## Credits & Attribution

### 3D Models
- **Cloth Ghost Model**: [Cloth Ghost by ANDRE](https://sketchfab.com/3d-models/cloth-ghost-f4fc1cd448c04a809d106975cfa7011b) on Sketchfab
  - License: CC Attribution (Creative Commons Attribution)

## License

This project is part of MSc Computer Science at University of the Arts London (UAL) by Natalia Bednarova as part of my thesis research.
