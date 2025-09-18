# Master Motion Capture System
# Processes video and depth data to generate a 3D trajectory for Unity.
# Supports a full debug mode with visualizations and an automated `mocap` mode.

import json
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
import mediapipe as mp
import os
from typing import List, Dict, Any, Optional, Tuple

class MasterMotionCapture:
    """Combines MediaPipe pose detection with depth data to create 3D motion trajectories."""
    
    def __init__(self, fx: float = 1478.49094, fy: float = 1477.42987, 
                 cx: float = 952.359616, cy: float = 717.874269) -> None:
        # Initialize MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(
            static_image_mode=False, 
            model_complexity=2, 
            smooth_landmarks=True,
            enable_segmentation=False, 
            smooth_segmentation=True,
            min_detection_confidence=0.5, 
            min_tracking_confidence=0.5
        )
        
        # CALIBRATED camera parameters from config (with defaults for backward compatibility)
        self.fx = fx  # Focal length in x (pixels)
        self.fy = fy  # Focal length in y (pixels)
        self.cx = cx  # Principal point x (center of image)
        self.cy = cy  # Principal point y (center of image)
        
        # Camera matrix for undistortion
        self.camera_matrix = np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ])
        
        # Distortion coefficients from calibration
        self.dist_coeffs = np.array([[2.50210100e-01, -1.25853616e+00, -1.27227784e-03, -3.50019815e-03, 2.09150240e+00]])
        
        print("[INFO] Master Motion Capture System initialized")
        print(f"[INFO] Camera calibrated: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")
        print()
    
    def load_depth_data(self, depth_file_path: str) -> List[Dict[str, Any]]:
        """Load depth data from JSON, JSON Lines, or binary formats."""
        print("[INFO] Loading depth data...")
        
        # Check file extension to determine format
        file_extension = os.path.splitext(depth_file_path)[1].lower()
        
        if file_extension == '.bin':
            # New high-performance binary format
            depth_data = self.load_binary_depth_data(depth_file_path)
        else:
            # Legacy JSON formats
            depth_data = self.load_json_depth_data(depth_file_path)
        
        print(f"[INFO] Total frames loaded: {len(depth_data)}")
        print()
        return depth_data
    
    def load_binary_depth_data(self, depth_file_path: str, width: int = 320, height: int = 240) -> List[Dict[str, Any]]:
        """Load depth data from optimized binary format (8-byte timestamp + float32 depth values)."""
        print("[INFO] Loading binary depth data...")
        depth_data = []
        
        # Each float is 4 bytes, timestamp is a double (8 bytes)
        bytes_per_frame = width * height * 4
        bytes_per_entry = 8 + bytes_per_frame
        
        try:
            with open(depth_file_path, 'rb') as f:
                # Check if file starts with "TEST_START" and skip it
                header_check = f.read(11)
                if header_check == b'TEST_START\n':
                    print("  [INFO] Detected TEST_START header, skipping 11 bytes...")
                else:
                    # Not a test header, go back to beginning
                    f.seek(0)
                
                frame_count = 0
                while True:
                    # Read the 8-byte timestamp
                    timestamp_bytes = f.read(8)
                    if not timestamp_bytes or len(timestamp_bytes) < 8:
                        break  # End of file or incomplete timestamp
                    
                    timestamp = np.frombuffer(timestamp_bytes, dtype=np.float64)[0]
                    
                    # Read the depth data bytes
                    depth_bytes = f.read(bytes_per_frame)
                    if len(depth_bytes) < bytes_per_frame:
                        print(f"[WARNING] Incomplete frame {frame_count} at end of file")
                        break  # Incomplete frame at end of file
                    
                    depth_array = np.frombuffer(depth_bytes, dtype=np.float32).reshape((height, width))
                    
                    frame_data = {
                        'timestamp': timestamp,
                        'width': width,
                        'height': height,
                        'depthValues': depth_array.tolist()  # Convert to list for compatibility
                    }
                    depth_data.append(frame_data)
                    frame_count += 1
                    
                    if frame_count % 50 == 0:
                        print(f"  Loaded {frame_count} binary frames...")
            
            print(f"[INFO] Loaded {len(depth_data)} depth frames from binary format")
            return depth_data
            
        except Exception as e:
            print(f"[ERROR] Error loading binary depth data: {e}")
            return []
    
    def load_json_depth_data(self, depth_json_path: str) -> List[Dict[str, Any]]:
        """Load depth data from JSON array or JSON Lines format."""
        print("[INFO] Loading JSON depth data...")
        
        depth_data = []
        with open(depth_json_path, 'r') as f:
            first_line = f.readline().strip()
            f.seek(0)  # Reset to beginning
            
            if first_line.startswith('['):
                # Old format: Regular JSON array
                depth_data = json.load(f)
                print(f"[INFO] Loaded {len(depth_data)} depth data frames from JSON array format")
            else:
                # New format: JSON Lines (one JSON object per line)
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    if line:  # Skip empty lines
                        try:
                            frame_data = json.loads(line)
                            depth_data.append(frame_data)
                        except json.JSONDecodeError as e:
                            print(f"[WARNING] Invalid JSON on line {line_num}: {e}")
                            continue
                print(f"[INFO] Loaded {len(depth_data)} depth frames from JSON Lines format")
        
        return depth_data
    
    def correct_for_camera_tilt(self, tracking_results: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Fits a line to the YZ plane to correct for camera pitch (up/down tilt)."""
        print("[INFO] Correcting for camera tilt...")
        
        valid_points = np.array([entry['point_3d'] for entry in tracking_results if entry['point_3d'] is not None])
        if len(valid_points) < 10:
            print("  [WARNING] Not enough valid points to perform tilt correction. Skipping.")
            return tracking_results

        # The slope of the YZ plot corresponds to the tangent of the pitch angle
        y_coords = valid_points[:, 1]
        z_coords = valid_points[:, 2]

        # Perform linear regression (fit a line to the YZ points)
        slope, intercept = np.polyfit(z_coords, y_coords, 1)

        # The slope of this line is the tangent of the camera's pitch angle
        pitch_angle = np.arctan(slope)
        print(f"  Detected camera pitch angle: {np.degrees(pitch_angle):.2f} degrees")

        # Counteract camera pitch by rotating around X-axis
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(-pitch_angle), -np.sin(-pitch_angle)],
            [0, np.sin(-pitch_angle), np.cos(-pitch_angle)]
        ])

        corrected_count = 0
        for entry in tracking_results:
            if entry['point_3d'] is not None:
                original_point = np.array(entry['point_3d'])
                corrected_point = rotation_matrix.dot(original_point)
                entry['point_3d'] = corrected_point.tolist()
                corrected_count += 1
                
        print(f"  [INFO] Applied tilt correction to {corrected_count} points.")
        return tracking_results
    
    def unproject_2d_to_3d(self, pixel_point: Tuple[int, int], depth: float) -> np.ndarray:
        """Convert 2D pixel coordinates to 3D world coordinates using pinhole camera model."""
        x_image, y_image = pixel_point
        
        X = depth * (x_image - self.cx) / self.fx
        Y = depth * (y_image - self.cy) / self.fy
        Z = depth
        
        return np.array([X, Y, Z])
    
    def create_depth_animation(self, depth_data, output_path, fps=30):
        """Create animated depth visualization"""
        print("[INFO] Creating depth animation...")
        
        # Get depth dimensions from first frame
        first_frame = depth_data[0]
        depth_width = first_frame.get('width', 320)
        depth_height = first_frame.get('height', 240)
        
        print(f"  Depth dimensions: {depth_width}x{depth_height}")
        
        import tempfile
        import os
        import subprocess
        
        temp_dir = tempfile.mkdtemp(prefix="depth_frames_")
        print(f"  Using temporary directory: {temp_dir}")
        
        for i, depth_frame in enumerate(depth_data):
            # Extract depth values array
            depth_values = depth_frame.get('depthValues', [])
            if not depth_values:
                continue
                
            # Convert to numpy array and reshape
            depth_array = np.array(depth_values).reshape(depth_height, depth_width)
            
            # Rotate 90 degrees clockwise to fix LiDAR orientation
            depth_array = np.rot90(depth_array, k=-1)  # k=-1 for 90 degrees clockwise
            
            # Filter out invalid depth values (0 or negative)
            valid_mask = depth_array > 0
            valid_depths = depth_array[valid_mask]
            
            if len(valid_depths) > 0:
                # Normalize to 0-255 range
                min_depth = np.min(valid_depths)
                max_depth = np.max(valid_depths)
                depth_range = max_depth - min_depth
                
                if depth_range > 0:
                    # Normalize and invert (closer = brighter)
                    normalized = (depth_array - min_depth) / depth_range
                    normalized = 1.0 - normalized  # Invert so closer objects are brighter
                    grayscale = (normalized * 255).astype(np.uint8)
                else:
                    grayscale = np.zeros((depth_height, depth_width), dtype=np.uint8)
                
                # Set invalid depths to black
                grayscale[~valid_mask] = 0
            else:
                grayscale = np.zeros((depth_height, depth_width), dtype=np.uint8)
            
            # Save frame as PNG
            frame_path = os.path.join(temp_dir, f"frame_{i:04d}.png")
            cv2.imwrite(frame_path, grayscale)
            
            if i % 50 == 0:
                print(f"  Processed frame {i+1}/{len(depth_data)}")
        
        # Use FFmpeg to create video from frames
        try:
            frame_pattern = os.path.join(temp_dir, "frame_%04d.png")
            cmd = [
                'ffmpeg', '-y',  # Overwrite output
                '-framerate', str(fps),
                '-i', frame_pattern,
                '-c:v', 'libx264',
                '-preset', 'ultrafast',  # Better compatibility
                '-crf', '18',  # Higher quality
                '-pix_fmt', 'yuv420p',
                '-movflags', '+faststart',  # Better streaming compatibility
                output_path
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode != 0:
                print(f"FFmpeg error: {result.stderr}")
                
        except FileNotFoundError:
            print("FFmpeg not found. Please install FFmpeg to create videos.")
        except Exception as e:
            print(f"Error creating video: {e}")
        finally:
            import shutil
            shutil.rmtree(temp_dir, ignore_errors=True)
    
    def get_or_create_video_timestamps(self, video_path):
        """Extract video frame timestamps, using cache if available."""
        cache_path = os.path.splitext(video_path)[0] + '_timestamps.json'
        
        if os.path.exists(cache_path):
            with open(cache_path, 'r') as f:
                return json.load(f)

        print("Creating video timestamp cache...")
        
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"Error: Could not open video file: {video_path}")
            return None
            
        timestamps = []
        frame_count = 0
        while True:
            ret, _ = cap.read()
            if not ret:
                break
            
            # Get timestamp in seconds
            ts = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            timestamps.append(ts)
            frame_count += 1
            
            if frame_count % 300 == 0:
                print(f"   ...processed {frame_count} frames...")

        cap.release()
        
        with open(cache_path, 'w') as f:
            json.dump(timestamps, f)
            
        return timestamps

    def process_video_with_mediapipe(self, video_path, depth_data):
        """Process video with MediaPipe and map to depth data using timestamp synchronization."""
        print("[INFO] Processing video with MediaPipe (Timestamp Sync Mode)...")
        
        video_timestamps = self.get_or_create_video_timestamps(video_path)
        if video_timestamps is None:
            return None
        video_timestamps_np = np.array(video_timestamps)
        
        # Open video
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"Error: Could not open video file: {video_path}")
            return None
        
        # Get video properties for scaling
        fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"  Video: {len(video_timestamps)} frames at ~{fps:.1f} FPS")
        print(f"  Depth data: {len(depth_data)} frames")
        
        scale_x = depth_data[0]['width'] / 1920
        scale_y = depth_data[0]['height'] / 1440
        print(f"  Scaling factors: x={scale_x:.3f}, y={scale_y:.3f}")
        
        tracking_results = []
        frames_with_depth = 0
        
        # --- NORMALIZE DEPTH TIMESTAMPS TO MATCH VIDEO RELATIVE TIME ---
        depth_timestamps = np.array([frame['timestamp'] for frame in depth_data])
        depth_start_time = depth_timestamps[0]
        relative_depth_timestamps = depth_timestamps - depth_start_time
        
        # Process each depth frame
        for depth_idx, depth_frame in enumerate(depth_data):
            # Get depth map
            depth_array = np.array(depth_frame['depthValues']).reshape(
                depth_frame['height'], depth_frame['width']
            )
            depth_array = np.rot90(depth_array, k=-1)
            
            depth_timestamp = relative_depth_timestamps[depth_idx]
            
            # Find the index of the video frame with the smallest time difference
            time_diffs = np.abs(video_timestamps_np - depth_timestamp)
            video_frame_idx = np.argmin(time_diffs)
            min_time_diff_ms = time_diffs[video_frame_idx] * 1000
            
            # Read the accurately synchronized video frame
            cap.set(cv2.CAP_PROP_POS_FRAMES, video_frame_idx)
            ret, frame = cap.read()
            
            if not ret:
                continue
            
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            rgb_frame = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2RGB)
            results = self.pose.process(rgb_frame)
            
            if results.pose_landmarks:
                nose_landmark = results.pose_landmarks.landmark[0]
                height, width = frame.shape[:2]
                nose_x = int(nose_landmark.x * width)
                nose_y = int(nose_landmark.y * height)
                
                depth_x = int(nose_x * scale_x)
                depth_y = int(nose_y * scale_y)
                
                depth_value = self.get_depth_value(depth_array, depth_x, depth_y)
                
                point_3d = self.unproject_2d_to_3d((nose_x, nose_y), depth_value) if depth_value is not None else None
                
                result = {
                    'depth_frame_idx': depth_idx,
                    'video_frame_idx': int(video_frame_idx),
                    'time_sync_diff_ms': min_time_diff_ms,
                    'nose_coords': [nose_x, nose_y],
                    'depth_coords': [depth_x, depth_y],
                    'depth_value': depth_value,
                    'point_3d': point_3d.tolist() if point_3d is not None else None
                }
                
                tracking_results.append(result)
                
                if depth_value is not None:
                    frames_with_depth += 1
                    if frames_with_depth <= 10 or frames_with_depth % 50 == 0:
                        print(f"  Depth frame {depth_idx} -> Video frame {video_frame_idx} (sync diff: {min_time_diff_ms:.1f}ms) -> Depth: {depth_value:.2f}m")
                else:
                    if len(tracking_results) % 50 == 0:
                         print(f"  Depth frame {depth_idx} -> Video frame {video_frame_idx} (sync diff: {min_time_diff_ms:.1f}ms) -> No depth data")
        
        cap.release()
        print(f"MediaPipe processing complete: {len(tracking_results)} tracked frames")
        
        return tracking_results
    
    def get_depth_value(self, depth_map, x, y):
        """Get depth value at given coordinates."""
        if 0 <= x < depth_map.shape[1] and 0 <= y < depth_map.shape[0]:
            depth_value = depth_map[y, x]
            if depth_value > 0:  # Valid depth value
                return depth_value
        return None
    
    def create_alignment_check(self, tracking_results, video_path, depth_data, output_path):
        """Create alignment check visualization showing video and depth frame correspondence."""
        print("[INFO] Creating alignment check visualization...")
        
        # Open video for frame extraction
        cap = cv2.VideoCapture(video_path)
        
        # Calculate grid layout - we need 2 subplots per frame (video + depth)
        n_frames = len(tracking_results)
        total_subplots = n_frames * 2
        
        # Calculate optimal grid size
        cols = int(np.ceil(np.sqrt(total_subplots)))
        rows = int(np.ceil(total_subplots / cols))
        
        print(f"  Creating visualization with {n_frames} frames in {rows}x{cols} grid...")
        print(f"  Total subplots: {total_subplots} (video + depth for each frame)")
        
        # Create figure with tight spacing
        fig, axes = plt.subplots(rows, cols, figsize=(cols*1.2, rows*0.8))
        fig.suptitle(f'Master Motion Capture: All Frames Alignment Check - {n_frames} Frames', fontsize=14, fontweight='bold')
        
        # Flatten axes for easier indexing
        if rows == 1:
            axes = axes.reshape(1, -1)
        axes = axes.flatten()
        
        # Process each tracked frame 
        for i, entry in enumerate(tracking_results):
            depth_idx = entry['depth_frame_idx']
            video_frame_idx = entry['video_frame_idx']
            nose_coords = entry['nose_coords']
            depth_coords = entry['depth_coords']
            depth_value = entry['depth_value']
            
            if i % 50 == 0:  # Print progress every 50 frames
                print(f"  Processing frame {i+1}/{n_frames}: Depth frame {depth_idx}, Video frame {video_frame_idx}")
            
            # Get depth frame
            depth_frame = depth_data[depth_idx]
            depth_array = np.array(depth_frame['depthValues']).reshape(
                depth_frame['height'], depth_frame['width']
            )
            depth_array = np.rot90(depth_array, k=-1)
            
            # Get video frame
            cap.set(cv2.CAP_PROP_POS_FRAMES, video_frame_idx)
            ret, video_frame = cap.read()
            
            if not ret:
                continue
            
            video_frame = cv2.rotate(video_frame, cv2.ROTATE_90_CLOCKWISE)
            
            # Plot video frame (left subplot)
            ax_video = axes[i*2]
            ax_video.imshow(cv2.cvtColor(video_frame, cv2.COLOR_BGR2RGB))
            ax_video.set_title(f'V{video_frame_idx}\nD{depth_idx}', fontsize=6)
            ax_video.axis('off')
            
            # Plot nose position on video (tiny dot)
            ax_video.scatter(nose_coords[0], nose_coords[1], c='red', s=8, marker='o',
                            edgecolors='white', linewidth=0.3)
            
            # Plot depth frame (right subplot)
            ax_depth = axes[i*2 + 1]
            im_depth = ax_depth.imshow(depth_array, cmap='viridis', aspect='equal')
            depth_title = f'{depth_value:.1f}m' if depth_value is not None else 'No depth'
            ax_depth.set_title(depth_title, fontsize=6)
            ax_depth.axis('off')
            
            # Plot transformed nose position on depth (tiny dot)
            ax_depth.scatter(depth_coords[0], depth_coords[1], c='red', s=8, marker='o',
                            edgecolors='white', linewidth=0.3)
        
        # Hide unused subplots
        for i in range(total_subplots, len(axes)):
            axes[i].axis('off')
        
        cap.release()
        
        # Set very tight layout with minimal spacing
        plt.tight_layout(pad=0.1, h_pad=0.05, w_pad=0.05)
        
        # Save plot
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Alignment check saved to: {output_path}")
    
    def create_3d_visualization(self, tracking_results, output_path, actual_duration=None):
        """Create animated 3D visualization with camera at origin."""
        print("[INFO] Creating 3D motion visualization...")
        
        # Extract 3D points (filter out None values)
        valid_3d_entries = [entry for entry in tracking_results if entry['point_3d'] is not None]
        if not valid_3d_entries:
            print("No valid 3D points found for visualization")
            return
        
        points_3d = np.array([entry['point_3d'] for entry in valid_3d_entries])
        
        # Create 3D plot
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Set up the plot
        ax.set_xlabel('X (meters) - Left/Right')
        ax.set_ylabel('Y (meters) - Up/Down')
        ax.set_zlabel('Z (meters) - Forward')
        ax.set_title('3D Motion Trajectory: Nose Movement', fontsize=14, fontweight='bold')
        
        # Calculate the actual range of motion to zoom in
        x_range = np.max(points_3d[:, 0]) - np.min(points_3d[:, 0])
        y_range = np.max(points_3d[:, 1]) - np.min(points_3d[:, 1])
        z_range = np.max(points_3d[:, 2]) - np.min(points_3d[:, 2])
        
        # Add padding around the motion range for better visibility
        padding_factor = 0.15  # 15% padding around the motion
        x_padding = x_range * padding_factor
        y_padding = y_range * padding_factor
        z_padding = z_range * padding_factor
        
        # Set axis limits to zoom in on the actual motion
        ax.set_xlim(
            np.min(points_3d[:, 0]) - x_padding,
            np.max(points_3d[:, 0]) + x_padding
        )
        ax.set_ylim(
            np.min(points_3d[:, 1]) - y_padding,
            np.max(points_3d[:, 1]) + y_padding
        )
        ax.set_zlim(
            np.min(points_3d[:, 2]) - z_padding,
            np.max(points_3d[:, 2]) + z_padding
        )
        ax.set_box_aspect([1, 1, 1])
        
        # Add camera model at origin
        camera_size = 0.05
        camera_corners = np.array([
            [-camera_size, -camera_size, 0],
            [camera_size, -camera_size, 0],
            [camera_size, camera_size, 0],
            [-camera_size, camera_size, 0],
            [-camera_size, -camera_size, camera_size],
            [camera_size, -camera_size, camera_size],
            [camera_size, camera_size, camera_size],
            [-camera_size, camera_size, camera_size]
        ])
        
        camera_faces = [
            [camera_corners[0], camera_corners[1], camera_corners[2], camera_corners[3]],
            [camera_corners[4], camera_corners[5], camera_corners[6], camera_corners[7]],
            [camera_corners[0], camera_corners[1], camera_corners[5], camera_corners[4]],
            [camera_corners[2], camera_corners[3], camera_corners[7], camera_corners[6]],
            [camera_corners[0], camera_corners[3], camera_corners[7], camera_corners[4]],
            [camera_corners[1], camera_corners[2], camera_corners[6], camera_corners[5]]
        ]
        
        camera_poly = Poly3DCollection(camera_faces, alpha=0.4, facecolor='red', edgecolor='black')
        ax.add_collection3d(camera_poly)
        
        # Initialize scatter plot and line
        scatter = ax.scatter([], [], [], c=[], cmap='viridis', s=50, alpha=0.8)
        line, = ax.plot([], [], [], 'b-', linewidth=2, alpha=0.7)
        
        # Add legend
        ax.scatter([], [], [], c='red', s=100, marker='s', label='Camera (Origin)')
        ax.legend()
        
        def animate(frame):
            # Show points up to current frame
            current_points = points_3d[:frame+1]
            
            if len(current_points) > 0:
                # Update scatter
                scatter._offsets3d = (current_points[:, 0], current_points[:, 1], current_points[:, 2])
                
                # Color by depth (Z coordinate)
                scatter.set_array(current_points[:, 2])
                
                # Update line
                if len(current_points) > 1:
                    line.set_data(current_points[:, 0], current_points[:, 1])
                    line.set_3d_properties(current_points[:, 2])
                
                # Add text showing current position
                if frame < len(points_3d):
                    current_pos = points_3d[frame]
                    ax.text2D(0.02, 0.98, 
                             f'Frame: {frame+1}/{len(points_3d)}\nX: {current_pos[0]:.2f}m\nY: {current_pos[1]:.2f}m\nZ: {current_pos[2]:.2f}m', 
                             transform=ax.transAxes, fontsize=10, 
                             bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
            
            return scatter, line
        
        # Create animation - use actual duration if provided, otherwise fallback to approximation
        if actual_duration is not None and actual_duration > 0:
            video_duration = actual_duration
            print(f"  Using actual data duration: {video_duration:.2f} seconds")
        else:
            video_duration = 10  # seconds (fallback approximation)
            print(f"  Using fallback duration: {video_duration} seconds")
        
        animation_interval = int((video_duration * 1000) / len(points_3d))  # milliseconds per frame
        
        anim = animation.FuncAnimation(fig, animate, frames=len(points_3d), 
                                     interval=animation_interval, blit=False, repeat=True)
        
        # Save animation with higher FPS for smoother playback
        anim.save(output_path, writer='ffmpeg', fps=20, dpi=100)
        print(f"3D visualization saved to: {output_path}")
        
        plt.show()
    
    def create_mediapipe_overlay_video(self, video_path, output_path):
        """Create video with MediaPipe pose skeleton overlay for debugging."""
        print("[INFO] Creating MediaPipe overlay video for debugging...")
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"Error opening video: {video_path}")
            return

        # Get video properties from the raw landscape video
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)

        # The output video will be portrait, so dimensions are swapped
        output_width, output_height = frame_height, frame_width 

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path, fourcc, fps, (output_width, output_height))
        
        frame_idx = 0
        while cap.isOpened():
            ret, frame = cap.read() # Reads the raw LANDSCAPE frame
            if not ret:
                break

            # 1. Undistort the original LANDSCAPE frame first for geometric accuracy
            frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            
            # 2. Run MediaPipe on the undistorted LANDSCAPE frame
            rgb_frame = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2RGB)
            results = self.pose.process(rgb_frame)
            
            # 3. Draw the skeleton on a copy of the LANDSCAPE frame
            frame_with_overlay = frame_undistorted.copy()
            if results.pose_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame_with_overlay,
                    results.pose_landmarks,
                    self.mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=4),
                    connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=2, circle_radius=2)
                )

            # 4. NOW, rotate the final frame (with the skeleton on it) to portrait for viewing
            frame_final = cv2.rotate(frame_with_overlay, cv2.ROTATE_90_CLOCKWISE)
            
            # 5. Write the correctly oriented frame to the output video
            out.write(frame_final)

            if frame_idx > 0 and frame_idx % 100 == 0:
                print(f"  Processed {frame_idx} frames for overlay video...")
            frame_idx += 1

        cap.release()
        out.release()
        print(f"MediaPipe overlay video saved to {output_path}")
    
    def run_complete_pipeline(self, video_path, depth_json_path, output_dir="output"):
        """Run the complete motion capture pipeline with visualizations."""
        print("[INFO] Starting complete motion capture pipeline")
        print("=" * 60)
        
        os.makedirs(output_dir, exist_ok=True)
        
        mediapipe_overlay_path = os.path.join(output_dir, "master_mediapipe_overlay.mp4")
        self.create_mediapipe_overlay_video(video_path, mediapipe_overlay_path)
        
        depth_data = self.load_depth_data(depth_json_path)
        
        # Step 1: Create depth animation with matching duration
        depth_animation_path = os.path.join(output_dir, "master_depth_animation.mp4")
        
        # Calculate duration from depth data timestamps (more accurate than video metadata)
        first_timestamp = depth_data[0]['timestamp']
        last_timestamp = depth_data[-1]['timestamp']
        actual_duration = last_timestamp - first_timestamp
        
        if actual_duration > 0:
            depth_fps = len(depth_data) / actual_duration
        else:
            depth_fps = 30  # Fallback
        
        self.create_depth_animation(depth_data, depth_animation_path, fps=depth_fps)
        
        # Step 2: Process video with MediaPipe and map to depth
        tracking_results = self.process_video_with_mediapipe(video_path, depth_data)
        
        if tracking_results:
            # Correct the entire trajectory for camera tilt
            tracking_results = self.correct_for_camera_tilt(tracking_results)
            
            # Save tracking results
            tracking_path = os.path.join(output_dir, "master_tracking_results.json")
            with open(tracking_path, 'w') as f:
                json.dump(tracking_results, f, indent=2)
            
            # Step 3: Create alignment check (limit to first 1/3 for faster processing)
            alignment_path = os.path.join(output_dir, "master_alignment_check.png")
            limited_results = tracking_results[:len(tracking_results)//3] if len(tracking_results) > 3 else tracking_results
            self.create_alignment_check(limited_results, video_path, depth_data, alignment_path)
            
            # Step 4: Create 3D visualization
            visualization_path = os.path.join(output_dir, "master_3d_visualization.mp4")
            self.create_3d_visualization(tracking_results, visualization_path, actual_duration)
            
            print("PIPELINE COMPLETE!")
            print(f"Results saved to: {output_dir}")
            print(f"Total tracked frames: {len(tracking_results)}")
            
            return tracking_results
        else:
            print("Pipeline failed: No tracking results generated")
            return None

    def run_verification(self, usdz_model_path, camera_pose_path):
        """Load 3D model and camera pose to visually verify alignment (auto-converts USDZ to OBJ)."""
        print("[INFO] Running camera pose verification")
        print("-" * 50)
        
        try:
            import open3d as o3d
        except ImportError:
            print("Open3D not installed. Please install with: pip install open3d")
            return False
        
        # Convert .usdz to .obj if necessary
        model_to_load = usdz_model_path
        if usdz_model_path.endswith('.usdz'):
            obj_path = usdz_model_path.replace('.usdz', '.obj')
            if not self._convert_usdz_to_obj(usdz_model_path, obj_path):
                print("Automatic conversion failed. Please convert to .obj manually.")
                return False
            model_to_load = obj_path

        # Load the room model
        try:
            room_mesh = o3d.io.read_triangle_mesh(model_to_load)
            if len(room_mesh.vertices) == 0:
                print("Room model appears to be empty")
                return False
            room_mesh.compute_vertex_normals()
        except Exception as e:
            print(f"Failed to load room model: {e}")
            return False

        # Load the camera pose data
        try:
            with open(camera_pose_path, 'r') as f:
                pose_data = json.load(f)
            
            position = np.array(pose_data["position"])
            quat_xyzw = pose_data["rotation_quaternion"]
            # Open3D expects quaternions in [w, x, y, z] format
            rotation_quat = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        except Exception as e:
            print(f"Failed to load or parse pose data: {e}")
            return False

        # Create and transform the camera marker
        camera_marker = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
        
        # Apply the +90 degree Z-axis correction for ARKit alignment
        correction_angle_rad = np.deg2rad(90)
        cos_a, sin_a = np.cos(correction_angle_rad), np.sin(correction_angle_rad)
        correction_matrix = np.array([[cos_a, -sin_a, 0], [sin_a, cos_a, 0], [0, 0, 1]])
        camera_marker.rotate(correction_matrix, center=(0, 0, 0))
        
        # Apply the final pose from the JSON file
        transform_matrix = np.identity(4)
        transform_matrix[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion(rotation_quat)
        transform_matrix[:3, 3] = position
        camera_marker.transform(transform_matrix)
        
        # Visualize
        print("Launching visualizer...")
        print("Close the visualizer window when done.")
        
        try:
            o3d.visualization.draw_geometries([room_mesh, camera_marker])
            print("Verification completed successfully!")
            return True
        except Exception as e:
            print(f"Visualization failed: {e}")
            return False

    def _convert_usdz_to_obj(self, usdz_path, obj_path):
        """Convert USDZ file to OBJ using Blender if available."""
        if not os.path.exists(usdz_path):
            return False
        
        if os.path.exists(obj_path):
            return True
        
        # Check if Blender is available
        blender_paths = [
            "/Applications/Blender.app/Contents/MacOS/Blender",
            "/usr/local/bin/blender",
            "blender"
        ]
        
        blender_cmd = None
        for path in blender_paths:
            try:
                import subprocess
                result = subprocess.run([path, "--version"], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    blender_cmd = path
                    break
            except:
                continue
        
        if not blender_cmd:
            print("Blender not found. Please convert USDZ to OBJ manually or install Blender.")
            return False
        
        # Create a Blender Python script for conversion
        blender_script = f"""
import bpy
import sys

# Clear existing mesh
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Import USDZ
try:
    bpy.ops.wm.usd_import(filepath='{usdz_path}')
    print("USDZ imported successfully")
except:
    print("Failed to import USDZ")
    sys.exit(1)

# Export as OBJ
try:
    bpy.ops.wm.obj_export(filepath='{obj_path}')
    print("Exported to OBJ successfully")
except:
    print("Failed to export OBJ")
    sys.exit(1)
"""
        
        # Write script to temporary file
        script_file = "temp_blender_script.py"
        with open(script_file, 'w') as f:
            f.write(blender_script)
        
        try:
            import subprocess
            # Run Blender with the script
            cmd = [blender_cmd, "--background", "--python", script_file]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            
            # Clean up
            if os.path.exists(script_file):
                os.remove(script_file)
            
            if result.returncode == 0 and os.path.exists(obj_path):
                return True
            else:
                print(f"Blender conversion failed: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            print("Blender conversion timed out")
            return False
        except Exception as e:
            print(f"Blender conversion error: {e}")
            return False


def main() -> None:
    """Main entry point supporting mocap, verify, and auto modes with optional headless processing."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Master Motion Capture & Camera Verification Tool")
    parser.add_argument(
        'mode', 
        nargs='?',
        choices=['mocap', 'verify', 'auto'], 
        default='auto',
        help="Choose the operating mode: 'mocap' for motion capture, 'verify' for camera verification, or 'auto' for automatic detection (default)"
    )
    parser.add_argument(
        '--headless',
        action='store_true',
        help="Run in headless mode for automation (no plots or videos)."
    )
    args = parser.parse_args()
    
    try:
        from config import (VIDEO_PATH, DEPTH_JSON_PATH, OUTPUT_DIR, 
                           USDZ_MODEL_PATH, CAMERA_POSE_PATH, CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY)
    except ImportError:
        print("[WARNING] Config file not found, using default paths")
        # Define default paths here if needed, or exit
        return

    motion_capture = MasterMotionCapture(CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY)
    
    # Determine which mode to run
    if args.mode == 'auto' and not args.headless:
        if USDZ_MODEL_PATH and CAMERA_POSE_PATH and os.path.exists(USDZ_MODEL_PATH) and os.path.exists(CAMERA_POSE_PATH):
            mode = 'verify'
        else:
            mode = 'mocap'
    else:
        mode = args.mode

    if mode == 'verify':
        print("\n--- RUNNING CAMERA POSE VERIFICATION ---")
        motion_capture.run_verification(USDZ_MODEL_PATH, CAMERA_POSE_PATH)
    
    elif mode == 'mocap':
        if args.headless:
            print("[INFO] Running headless motion capture for Unity integration")
            os.makedirs(OUTPUT_DIR, exist_ok=True)
            
            # 1. Convert USDZ to OBJ if needed
            obj_path = USDZ_MODEL_PATH.replace('.usdz', '.obj')
            if motion_capture._convert_usdz_to_obj(USDZ_MODEL_PATH, obj_path):
                print(f"[INFO] Model ready at: {obj_path}")
            
            # 2. Copy room model files to StreamingAssets
            import shutil
            if os.path.exists(obj_path):
                obj_dest = os.path.join(OUTPUT_DIR, "room_model.obj")
                shutil.copy2(obj_path, obj_dest)
                print(f"[INFO] Copied room model to: {obj_dest}")
                
                # Copy MTL file if it exists
                mtl_path = obj_path.replace('.obj', '.mtl')
                if os.path.exists(mtl_path):
                    mtl_dest = os.path.join(OUTPUT_DIR, "room_model.mtl")
                    shutil.copy2(mtl_path, mtl_dest)
                    print(f"[INFO] Copied material file to: {mtl_dest}")
            
            # 3. Copy camera pose to StreamingAssets
            if os.path.exists(CAMERA_POSE_PATH):
                pose_dest = os.path.join(OUTPUT_DIR, "camera_pose.json")
                shutil.copy2(CAMERA_POSE_PATH, pose_dest)
                print(f"[INFO] Copied camera pose to: {pose_dest}")
            
            # 4. Run core motion capture process
            depth_data = motion_capture.load_depth_data(DEPTH_JSON_PATH)
            tracking_results = motion_capture.process_video_with_mediapipe(VIDEO_PATH, depth_data)
            
            # 5. Save the final raw trajectory file
            if tracking_results:
                tracking_results = motion_capture.correct_for_camera_tilt(tracking_results)
                tracking_path = os.path.join(OUTPUT_DIR, "master_tracking_results.json")
                with open(tracking_path, 'w') as f:
                    json.dump(tracking_results, f, indent=2)
                print(f"[INFO] Raw trajectory saved to: {tracking_path}")
                print("[INFO] Headless processing complete!")
            else:
                print("[ERROR] Headless processing failed!")
        else:
            # This is the full debug workflow for manual runs
            print("[INFO] Running full debug motion capture")
            motion_capture.run_complete_pipeline(VIDEO_PATH, DEPTH_JSON_PATH, OUTPUT_DIR)

if __name__ == "__main__":
    main()
