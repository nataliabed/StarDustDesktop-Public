// MemoryPlayer.cs
// Main controller for the StarDust visualization scene.
// Handles Python script automation, data loading, animation, and user interaction.

using UnityEngine;
using UnityEngine.VFX;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using Newtonsoft.Json;
using Dummiesman;


// Using PoseData to match raw camera_pose.json
[System.Serializable]
public class PoseData {
    public float[] rotation_quaternion; // [x, y, z, w]
    public float[] position;            // [x, y, z]
}

public class RawTrajectoryPoint {
    // We only care about the final 3D point for now, but this structure can be expanded
    public List<float> point_3d;
}

public class MemoryPlayer : MonoBehaviour {
    #region Public Fields & Properties
    [Header("Scene Objects")]
    public Camera mainCamera;
    
    public GameObject personPrefab;
    
    public Material ghostlyRoomMaterial;
    
    public VisualEffect roomPointCloudVFX;

    [Header("Trajectory Settings")]
    [Tooltip("Scale multiplier for the trajectory's X and Y axes.")]
    public float trajectoryXYScale = 1.0f;
    
    [Tooltip("Scale multiplier for the trajectory's Z axis.")]
    public float trajectoryZScale = 1.0f;
    
    [Tooltip("Lock the Y-axis to a fixed height to prevent unrealistic vertical jumping.")]
    public bool lockYAxis = false;
    
    [Tooltip("The fixed height (Y position) to use when Y-axis is locked.")]
    public float fixedYPosition = 0.45f;

    [Header("Trajectory Adjustments")]
    [Tooltip("Rotates the entire trajectory path left or right around the camera's viewpoint.")]
    [Range(-180f, 180f)]
    public float trajectoryYawRotation = 10f;
    
    [Header("Ghost Movement")]
    [Tooltip("How many intermediate steps to create between trajectory points (higher = smoother).")]
    [Range(1, 10)]
    public int interpolationSteps = 10;
    
    [Header("Ghost Effects")]
    [Tooltip("Enable a subtle up-and-down bobbing motion.")]
    public bool enableHover = true;
    [Tooltip("How fast the ghost bobs up and down.")]
    public float hoverSpeed = 1.5f;
    [Tooltip("How high and low the ghost bobs.")]
    public float hoverAmplitude = 0.05f;
    [Tooltip("Enable a subtle, slow rotation.")]
    public bool enableSpin = true;
    [Tooltip("How fast the ghost spins (degrees per second).")]
    public float spinSpeed = 50f;
    
    [Header("Room Point Cloud")]
    [Tooltip("Render the room as a point cloud.")]
    public bool usePointCloud = true;
    [Tooltip("Number of particles to spawn.")]
    [Range(1000, 200000)]
    public int particleCount = 200000;
    [Tooltip("Skybox material for solid model mode.")]
    public Material solidModelSkybox;
    [Tooltip("Skybox material for point cloud mode (usually black).")]
    public Material pointCloudSkybox;

    [Header("Camera Position Visualization")]
    [Tooltip("Show/hide the camera position orb and coordinate axes for verification.")]
    public bool showCameraPosition = false;
    
    [Header("Camera Controls")]
    [Tooltip("Enable mouse orbit controls for the camera.")]
    public bool enableOrbitControls = true;
    [Tooltip("Point in space to orbit around (center of the room).")]
    public Vector3 orbitCenter = new Vector3(0, 0, 0);
    [Tooltip("Distance from the orbit center.")]
    public float orbitDistance = 3.0f;
    [Tooltip("Mouse sensitivity for rotation.")]
    public float mouseSensitivity = 50.0f;
    [Tooltip("Zoom sensitivity for scroll wheel.")]
    public float zoomSensitivity = 10.0f;
    [Tooltip("Minimum distance from orbit center.")]
    public float minDistance = 0.5f;
    [Tooltip("Maximum distance from orbit center.")]
    public float maxDistance = 15.0f;
    
    [Header("Audio Settings")]
    [Tooltip("Background music audio clip.")]
    public AudioClip backgroundMusic;
    [Tooltip("Ghost shimmer sound audio clip.")]
    public AudioClip shimmerSound;
    [Tooltip("Background music volume.")]
    [Range(0f, 1f)]
    public float backgroundMusicVolume = 0.3f;
    [Tooltip("Ghost shimmer volume.")]
    [Range(0f, 1f)]
    public float shimmerVolume = 0.3f;
    [Tooltip("Distance at which shimmer sound reaches maximum volume.")]
    public float shimmerMaxDistance = 2.0f;
    [Tooltip("Distance at which shimmer sound starts to fade in.")]
    public float shimmerMinDistance = 8.0f;
    
    [Header("Automation Settings")]
    [Tooltip("Path to the master_motion_capture.py script, relative to the Assets folder.")]
    public string pythonScriptPath = "Scripts/master_motion_capture.py";
    
    private string pythonExecutablePath = "/opt/anaconda3/bin/python";

    
    [Header("Playback Settings")]
    [Tooltip("Speed multiplier for animation playback (1.0 = normal speed, 2.0 = double speed, 0.5 = half speed).")]
    [Range(0.1f, 5.0f)]
    public float playbackSpeed = 2.0f;
    public float sourceVideoFPS = 30.0f;
    
    [Header("Trail Settings")]
    public float personScale = 0.1f;
    public float trailPointSize = 0.02f; // Size of trail points
    public Color trailColor = Color.white;
    public bool createTrailPoints = true;
    
    [Header("Ghost Effect Settings")]
    public bool makePersonGhostly = true; // Add ghost effect to person
    public Material ghostMaterial;
    public bool useWallMaterialForGhost = true;
    public Color ghostColor = Color.white;
    public float ghostTransparency = 0.7f;
    
    [Header("Ghost Particle Effects")]
    [Tooltip("Enable falling particles from the ghost's surface.")]
    public bool enableFallingParticles = true;
    [Tooltip("Particle system for the falling effect.")]
    public ParticleSystem fallingParticles;
    [Tooltip("Number of particles to emit per second.")]
    [Range(10, 1000)]
    public int particlesPerSecond = 30;
    [Tooltip("Speed of falling particles (gravity effect).")]
    [Range(0.1f, 10.0f)]
    public float particleFallSpeed = 2.0f;
    [Tooltip("Color of the falling particles.")]
    public Color particleColor = new Color(0.95f, 0.95f, 0.95f, 1.0f);
    [Tooltip("Size of the falling particles.")]
    [Range(0.001f, 0.1f)]
    public float particleSize = 0.005f;
    #endregion
    
    #region Private State Variables
    // UI Settings
    private bool enableUI = true;
    private Canvas uiCanvas;
    private Toggle pointCloudToggle;
    private bool enablePlaybackControls = true;
    private float playbackPanelHeight = 80f;
    
    // Stores the final, converted world transform for trajectory calculations
    private Vector3 cameraWorldPosition;
    private Quaternion cameraWorldRotation;
    private List<GameObject> trailPoints = new List<GameObject>();
    private GameObject cameraMarker; // Reference to the camera position orb
    private GameObject currentRoomModel; // Reference to the current room model

    // --- VARIABLES TO TRACK CHANGES ---
    private List<RawTrajectoryPoint> loadedTrajectoryPoints; // Store the loaded data
    private Coroutine animationCoroutine; // A reference to the running animation
    private GameObject currentPersonInstance; // Reference to the current ghost instance
    private float lastXYScale; // To check if the slider value has changed
    private float lastZScale; // To check if the slider value has changed
    private bool lastLockYAxis; // To check if the toggle has changed
    private float lastFixedYPosition; // To check if the Y position has changed
    private float lastYawRotation; // Tracker for the rotation slider
    private Vector3 ghostCurrentPosition; // Current position of the ghost for smoothing
    
    // --- ORBIT CAMERA VARIABLES ---
    private float orbitX = 0f; // Horizontal rotation
    private float orbitY = 0f; // Vertical rotation
    private bool isDragging = false; // Track if mouse is being dragged
    
    // --- PLAYBACK CONTROL VARIABLES ---
    private bool isPlaying = true; // Track if animation is playing (start as playing)
    private bool isPaused = false; // Track if animation is paused
    private bool isScrubbing = false; // Track if user is scrubbing the timeline
    private bool wasPlayingBeforeScrub = false; // Remember if we were playing before scrubbing
    private bool lastPlayState = true; // Track last play state to avoid unnecessary icon updates
    private float currentTime = 0f; // Current playback time
    private float totalDuration = 0f; // Total animation duration
    private Slider timelineSlider; // Reference to timeline slider
    private Button playButton; // Reference to play/pause button
    private Button stopButton; // Reference to stop button
    private Text timeText; // Reference to time display text

    // --- AUDIO VARIABLES ---
    private AudioSource backgroundMusicSource; // Background music audio source
    private AudioSource shimmerAudioSource; // Ghost shimmer audio source
    private float originalBackgroundVolume; // Store original background volume for ducking
    
    // --- AUTOMATION VARIABLES ---
    private bool isProcessing = false; // Track if Python processing is running
    
    // --- THREAD-SAFE COMMUNICATION VARIABLES ---
    private System.Text.StringBuilder processOutput; // To collect all standard output
    private System.Text.StringBuilder processError;  // To collect all standard error
    
    // --- LOADING SCREEN VARIABLES ---
    private Canvas loadingCanvas; // Loading screen canvas
    private Text loadingText; // Loading status text
    #endregion

    #region Unity Lifecycle Methods
    IEnumerator Start()
    {
        // 1. Show loading screen first
        SetupLoadingScreen();
        UpdateLoadingText("Loading, processing, retrieving your memories...");
        
        // 2. Wait for Unity to finish initial asset importing
        UpdateLoadingText("Preparing system...");
        yield return new WaitForSeconds(3f);
        
        // 3. Check if files already exist, if not run Python script
        if (AreRequiredFilesPresent())
        {
            UpdateLoadingText("Found existing processed files, skipping Python processing...");
            yield return new WaitForSeconds(1f); // Show the message briefly
            isProcessing = false; // Mark as successful
        }
        else
        {
            // 4. Start the Python script and wait for it to finish
            yield return StartCoroutine(RunPythonProcessor());
        }

        // 3. Once the script is done, run the setup logic
        if (!isProcessing) // Only run if processing was successful
        {
            UpdateLoadingText("Setting up visualization...");
        LoadRoomModel();
        LoadAndSetCameraPose(); 
        LoadClothGhostPrefab();
        LoadAndPlayTrajectory();
            SetupAudio();
            if (enableUI) { SetupUI(); }
        if (enablePlaybackControls) {
            SetupPlaybackControls();
            StartCoroutine(StartInitialAnimation());
            }
            
            // 4. Hide loading screen and show the visualization
            HideLoadingScreen();
        }
        else
        {
            UpdateLoadingText("Processing failed. Check console for details.");
        }
    }

    // Update is called every frame. We use it to check for changes.
    void Update() {
        if (trajectoryXYScale != lastXYScale || 
            trajectoryZScale != lastZScale || 
            lockYAxis != lastLockYAxis || 
            fixedYPosition != lastFixedYPosition ||
            trajectoryYawRotation != lastYawRotation) {
            RestartTrajectoryAnimation();
        }
        
        // Update camera marker visibility if the toggle has changed
        if (cameraMarker != null) {
            cameraMarker.SetActive(showCameraPosition);
        }
        
        if (enableOrbitControls && mainCamera != null && !isScrubbing) {
            HandleOrbitCamera();
        }
        
        // Handle playback controls
        if (enablePlaybackControls) {
            HandlePlaybackControls();
        }
        
        // Update audio based on distance to ghost
        UpdateAudioBasedOnDistance();
    }
    #endregion

    #region Python Process Management
    IEnumerator RunPythonProcessor()
    {
        if (string.IsNullOrEmpty(pythonExecutablePath))
        {
            Debug.LogError("Python Executable Path is not set in the Inspector!");
            yield break;
        }

        isProcessing = true;
        Debug.Log("Starting Python script asynchronously...");

        processOutput = new System.Text.StringBuilder();
        processError = new System.Text.StringBuilder();

        string scriptFullPath = Path.Combine(Application.dataPath, pythonScriptPath);
        Process process = new Process();
        process.StartInfo.FileName = pythonExecutablePath;
        process.StartInfo.Arguments = $"\"{scriptFullPath}\" mocap --headless"; // Keep --headless for automation
        process.StartInfo.WorkingDirectory = Application.dataPath + "/Scripts"; // Set working directory to Scripts folder
        process.StartInfo.UseShellExecute = false;
        process.StartInfo.RedirectStandardOutput = true;
        process.StartInfo.RedirectStandardError = true;
        process.StartInfo.CreateNoWindow = true;
        process.EnableRaisingEvents = true;

        // Subscribe to async events to read process output without blocking the main thread
        process.OutputDataReceived += OnOutputDataReceived;
        process.ErrorDataReceived += OnErrorDataReceived;

        process.Start();
        process.BeginOutputReadLine();
        process.BeginErrorReadLine();

        while (!process.HasExited)
        {
            yield return null;
        }
        
        if (process.ExitCode == 0)
        {
            UpdateLoadingText("Processing complete!");
            Debug.Log("Python script finished successfully.");
            Debug.Log("--- Python Output ---\n" + processOutput.ToString());
            isProcessing = false;
        }
        else
        {
            UpdateLoadingText("Processing failed!");
            Debug.LogError("Python script failed with exit code: " + process.ExitCode);
            Debug.LogError("--- Python Error ---\n" + processError.ToString());
            Debug.LogError("--- Python Output ---\n" + processOutput.ToString());
        }
        
        process.OutputDataReceived -= OnOutputDataReceived;
        process.ErrorDataReceived -= OnErrorDataReceived;
    }
    
    private void OnOutputDataReceived(object sender, DataReceivedEventArgs e)
    {
        if (e.Data != null)
        {
            processOutput.AppendLine(e.Data);
            Debug.Log($"Python Output: {e.Data}");
        }
    }

    private void OnErrorDataReceived(object sender, DataReceivedEventArgs e)
    {
        if (e.Data != null)
        {
            processError.AppendLine(e.Data);
            
            // Check if it's a real error or just library initialization messages
            bool isRealError = !e.Data.Contains("warning") && !e.Data.Contains("WARNING") && 
                              !e.Data.Contains("info") && !e.Data.Contains("INFO") &&
                              !e.Data.Contains("GL version") && !e.Data.Contains("GL context") &&
                              !e.Data.Contains("TensorFlow Lite") && !e.Data.Contains("XNNPACK") &&
                              !e.Data.Contains("inference_feedback_manager") && !e.Data.Contains("Feedback manager") &&
                              !e.Data.Contains("absl::InitializeLog") && !e.Data.Contains("STDERR") &&
                              !e.Data.Contains("landmark_projection_calculator") && !e.Data.Contains("NORM_RECT") &&
                              !e.Data.Contains("IMAGE_DIMENSIONS") && !e.Data.Contains("PROJECTION_MATRIX");
            
            if (isRealError)
            {
                Debug.LogError($"Python Error: {e.Data}");
            }
            else
            {
                // Just log library initialization messages without changing the UI
                Debug.LogWarning($"Python Library Info: {e.Data}");
            }
        }
    }
    
    // Check if all required files are present in StreamingAssets
    bool AreRequiredFilesPresent() {
        string streamingAssetsPath = Application.streamingAssetsPath;
        
        // Check for required files
        string[] requiredFiles = {
            "room_model.obj",
            "camera_pose.json", 
            "master_tracking_results.json"
        };
        
        foreach (string fileName in requiredFiles) {
            string filePath = Path.Combine(streamingAssetsPath, fileName);
            if (!File.Exists(filePath)) {
                Debug.Log($"Missing file: {fileName}");
                return false;
            }
        }
        
        // Check for optional material file
        string mtlPath = Path.Combine(streamingAssetsPath, "room_model.mtl");
        if (File.Exists(mtlPath)) {
            Debug.Log("Found material file: room_model.mtl");
        } else {
            Debug.Log("Material file not found (optional): room_model.mtl");
        }
        
        Debug.Log("All required files found in StreamingAssets!");
        return true;
    }
    #endregion

    #region UI Management
    // Setup loading screen
    void SetupLoadingScreen() {
        // Create loading canvas
        GameObject loadingCanvasObj = new GameObject("Loading Canvas");
        loadingCanvas = loadingCanvasObj.AddComponent<Canvas>();
        loadingCanvas.renderMode = RenderMode.ScreenSpaceOverlay;
        loadingCanvas.sortingOrder = 1000; // On top of everything
        
        // Add CanvasScaler
        CanvasScaler scaler = loadingCanvasObj.AddComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920, 1080);
        
        // Add background
        GameObject backgroundObj = new GameObject("Loading Background");
        backgroundObj.transform.SetParent(loadingCanvas.transform, false);
        Image bgImage = backgroundObj.AddComponent<Image>();
        bgImage.color = new Color(0, 0, 0, 0.9f); // Dark background
        
        RectTransform bgRect = backgroundObj.GetComponent<RectTransform>();
        bgRect.anchorMin = Vector2.zero;
        bgRect.anchorMax = Vector2.one;
        bgRect.offsetMin = Vector2.zero;
        bgRect.offsetMax = Vector2.zero;
        
        // Add loading text
        GameObject textObj = new GameObject("Loading Text");
        textObj.transform.SetParent(loadingCanvas.transform, false);
        loadingText = textObj.AddComponent<Text>();
        loadingText.text = "Loading...";
        loadingText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        loadingText.fontSize = 32;
        loadingText.color = Color.white;
        loadingText.alignment = TextAnchor.MiddleCenter;
        
        RectTransform textRect = textObj.GetComponent<RectTransform>();
        textRect.anchorMin = new Vector2(0.1f, 0.4f);
        textRect.anchorMax = new Vector2(0.9f, 0.6f);
        textRect.offsetMin = Vector2.zero;
        textRect.offsetMax = Vector2.zero;
        
        Debug.Log("Loading screen setup complete");
    }
    
    // Update loading text
    void UpdateLoadingText(string message) {
        if (loadingText != null) {
            loadingText.text = message;
            Debug.Log($"Loading: {message}");
        }
    }
    
    // Hide loading screen
    void HideLoadingScreen() {
        if (loadingCanvas != null) {
            loadingCanvas.gameObject.SetActive(false);
            Debug.Log("Loading screen hidden");
        }
    }
    #endregion

    #region Data Loading & Initialization
    // Clean up existing room model to prevent stacking
    void CleanupExistingRoomModel() {
        if (currentRoomModel != null) {
            Destroy(currentRoomModel);
            currentRoomModel = null;
        }
        
        // Stop VFX only if it exists and we're switching modes
        if (roomPointCloudVFX != null) {
            roomPointCloudVFX.Stop();
        }
    }

    void LoadRoomModel()
    {
        Debug.Log("LoadRoomModel() called - starting room loading...");
        
        // Clean up any existing room model first (only if one exists)
        if (currentRoomModel != null) {
            CleanupExistingRoomModel();
        }
        
        string filePath = Path.Combine(Application.streamingAssetsPath, "room_model.obj");
        Debug.Log($"Looking for room model at: {filePath}");
        
        if (!File.Exists(filePath)) {
            Debug.LogError("room_model.obj not found!");
            return;
        }
        
        Debug.Log("room_model.obj found, loading...");

        var loadedObj = new OBJLoader().Load(filePath);
        Debug.Log($"OBJ loaded successfully: {loadedObj != null}");
        
        if (loadedObj == null) {
            Debug.LogError("Failed to load OBJ file!");
            return;
        }
        
        // Store reference to the loaded room model
        currentRoomModel = loadedObj;
        
        // Calculate and set the orbit center to the room's center
        CalculateRoomCenter(loadedObj);
        
        var originalRenderers = loadedObj.GetComponentsInChildren<MeshRenderer>();

        if (usePointCloud)
        {
            
            // Use VFX Graph for the twinkling point cloud effect
            if (roomPointCloudVFX == null) {
                Debug.LogError("The 'Room Point Cloud VFX' has not been assigned in the Inspector!");
                Debug.LogError("Please assign a VFX Graph to the 'Room Point Cloud VFX' slot in the Inspector.");
                return;
            }

            
            try {
                // Ensure VFX Graph is properly initialized
                if (!roomPointCloudVFX.gameObject.activeInHierarchy) {
                    roomPointCloudVFX.gameObject.SetActive(true);
                }
                
                // Stop any existing playback
                roomPointCloudVFX.Stop();
                
                // Combine ALL meshes from the room object to get the complete room
                var meshFilters = loadedObj.GetComponentsInChildren<MeshFilter>();
                if (meshFilters.Length == 0) {
                    Debug.LogError("No meshes found in the loaded room object!");
                    return;
                }
                
                
                // Create combined mesh with original transforms preserved
                CombineInstance[] combine = new CombineInstance[meshFilters.Length];
                for (int i = 0; i < meshFilters.Length; i++) {
                    combine[i].mesh = meshFilters[i].sharedMesh;
                    combine[i].transform = meshFilters[i].transform.localToWorldMatrix;
                }
                
                Mesh combinedMesh = new Mesh();
                combinedMesh.name = "CombinedRoomMesh";
                combinedMesh.CombineMeshes(combine);
                // Reset the VFX graph's transform to the world origin.
                // This ensures the world-space vertices in the combined mesh are rendered
                // in the correct absolute positions, matching the solid model.
                roomPointCloudVFX.transform.position = Vector3.zero;
                roomPointCloudVFX.transform.rotation = Quaternion.identity;
                roomPointCloudVFX.transform.localScale = Vector3.one;
                
                roomPointCloudVFX.SetMesh("RoomMesh", combinedMesh);
                
                // Use simple uniform particle distribution
                roomPointCloudVFX.SetInt("SpawnCount", particleCount);
                
                // Start the VFX Graph
                roomPointCloudVFX.Play();
            } catch (System.Exception e) {
                Debug.LogError($"Error setting VFX Graph properties: {e.Message}");
                Debug.LogError("Make sure your VFX Graph has these exposed properties:");
                Debug.LogError("- RoomMesh (Mesh)");
                Debug.LogError("- SpawnCount (Int)");
                Debug.LogError("And that the VFX Graph is properly configured with particle output");
            }
            
            // Hide the original solid model so only the twinkling points are visible
            foreach(var renderer in originalRenderers) {
                renderer.enabled = false;
            }
            
            // Change skybox for point cloud mode
            ChangeSkybox(pointCloudSkybox);
        }
        else
        {
            // Ensure the solid model is visible
            foreach (var renderer in originalRenderers) renderer.enabled = true;
            
            // Apply the ghostly material to the solid model
            if (ghostlyRoomMaterial != null) {
                foreach (var renderer in originalRenderers) {
                    renderer.material = ghostlyRoomMaterial;
                }
            }
            
            // Reset skybox to solid model mode
            ChangeSkybox(solidModelSkybox);
        }
    }
    
    

    void LoadClothGhostPrefab() {
        // Try to load the cloth ghost prefab automatically
        GameObject clothGhostPrefab = Resources.Load<GameObject>("ClothGhostPrefab");
        if (clothGhostPrefab != null) {
            personPrefab = clothGhostPrefab;
        } else {
            Debug.LogWarning("Cloth ghost not found. Please assign personPrefab manually in Inspector.");
        }
    }

    void LoadAndSetCameraPose() {
        // 1. Load the RAW camera pose file
        string filePath = Path.Combine(Application.streamingAssetsPath, "camera_pose.json");
        if (!File.Exists(filePath)) {
            Debug.LogError("camera_pose.json not found! Make sure you are using the raw data file.");
            return;
        }

        string json = File.ReadAllText(filePath);
        PoseData poseData = JsonConvert.DeserializeObject<PoseData>(json);

        // Convert from ARKit's right-handed system to Unity's left-handed system
        // Position conversion
        Vector3 arkitPosition = new Vector3(poseData.position[0], poseData.position[1], poseData.position[2]);
        Vector3 unityPosition = new Vector3(-arkitPosition.x, arkitPosition.y, arkitPosition.z);

        // Rotation conversion
        Quaternion arkitRotation = new Quaternion(
            poseData.rotation_quaternion[0], poseData.rotation_quaternion[1],
            poseData.rotation_quaternion[2], poseData.rotation_quaternion[3]);
        Quaternion unityHandednessRotation = new Quaternion(-arkitRotation.x, arkitRotation.y, arkitRotation.z, -arkitRotation.w);
        
        // Apply corrective rotations to align axes correctly
        Quaternion zAxisCorrection = Quaternion.Euler(0, 0, -90);
        Quaternion yAxisCorrection = Quaternion.Euler(0, 180, 0);
        Quaternion finalUnityRotation = unityHandednessRotation * zAxisCorrection * yAxisCorrection;

        // 3. Create the visual marker (the "orb")
        cameraMarker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        cameraMarker.name = "CameraPoseMarker";
        cameraMarker.transform.localScale = new Vector3(0.05f, 0.05f, 0.05f);
        Destroy(cameraMarker.GetComponent<SphereCollider>());
        
        // Position and rotate the marker
        cameraMarker.transform.position = unityPosition;
        cameraMarker.transform.rotation = finalUnityRotation;
        
        // Add the coordinate axes to the marker for visual confirmation
        CreateCoordinateAxes(cameraMarker.transform);
        
        // Set initial visibility based on the toggle
        cameraMarker.SetActive(showCameraPosition);

        // 4. Store the final pose in our class variables for the trajectory calculation
        cameraWorldPosition = unityPosition;
        cameraWorldRotation = finalUnityRotation;

    }
    
    // This helper function now attaches axes to any object we give it
    void CreateCoordinateAxes(Transform parent) {
        // X axis (red)
        GameObject xAxis = GameObject.CreatePrimitive(PrimitiveType.Cube);
        xAxis.name = "X_Axis";
        xAxis.transform.SetParent(parent, false); // Use false to respect local scale
        xAxis.transform.localPosition = new Vector3(0.5f, 0, 0);
        xAxis.transform.localScale = new Vector3(1.0f, 0.05f, 0.05f);
        xAxis.GetComponent<Renderer>().material.color = Color.red;
        Destroy(xAxis.GetComponent<Collider>());

        // Y axis (green)
        GameObject yAxis = GameObject.CreatePrimitive(PrimitiveType.Cube);
        yAxis.name = "Y_Axis";
        yAxis.transform.SetParent(parent, false);
        yAxis.transform.localPosition = new Vector3(0, 0.5f, 0);
        yAxis.transform.localScale = new Vector3(0.05f, 1.0f, 0.05f);
        yAxis.GetComponent<Renderer>().material.color = Color.green;
        Destroy(yAxis.GetComponent<Collider>());

        // Z axis (blue)
        GameObject zAxis = GameObject.CreatePrimitive(PrimitiveType.Cube);
        zAxis.name = "Z_Axis";
        zAxis.transform.SetParent(parent, false);
        zAxis.transform.localPosition = new Vector3(0, 0, 0.5f);
        zAxis.transform.localScale = new Vector3(0.05f, 0.05f, 1.0f);
        zAxis.GetComponent<Renderer>().material.color = Color.blue;
        Destroy(zAxis.GetComponent<Collider>());
    }

    void LoadAndPlayTrajectory() {
        string filePath = Path.Combine(Application.streamingAssetsPath, "master_tracking_results.json");
        if (!File.Exists(filePath)) {
            Debug.LogError("master_tracking_results.json not found!");
            return;
        }

        string json = File.ReadAllText(filePath);
        // Store the loaded points in our new class-level variable
        loadedTrajectoryPoints = JsonConvert.DeserializeObject<List<RawTrajectoryPoint>>(json);

        // Don't start animation here - let playback controls handle it
    }

    // This function handles stopping, clearing, and restarting the animation
    void RestartTrajectoryAnimation() {
        // 1. Stop the current animation if it is running
        if (animationCoroutine != null) {
            StopCoroutine(animationCoroutine);
            animationCoroutine = null;
        }

        // 2. Clear all the previously created trail points and person instance
        ClearTrailPoints();
        if (currentPersonInstance != null) {
            // Stop particle system before destroying
            if (fallingParticles != null) {
                fallingParticles.Stop();
            }
            Destroy(currentPersonInstance);
            currentPersonInstance = null;
        }

        // 3. Start the animation coroutine again from the beginning
        if (loadedTrajectoryPoints != null && loadedTrajectoryPoints.Count > 0) {
            // Only start animation if we're in playing state
            if (isPlaying && !isPaused) {
                // Use the playback control animation system
                animationCoroutine = StartCoroutine(AnimateTrajectoryFromFrame(loadedTrajectoryPoints, 0));
            }
        }

        // 4. Update our tracker variables to the new, current values
        lastXYScale = trajectoryXYScale;
        lastZScale = trajectoryZScale;
        lastLockYAxis = lockYAxis;
        lastFixedYPosition = fixedYPosition;
        lastYawRotation = trajectoryYawRotation;
    }

    
    // Helper function to calculate world position from a trajectory point
    Vector3 CalculateWorldPosition(RawTrajectoryPoint point, Quaternion yawRotation) {
        // 1. Get the raw local position from the JSON data
        Vector3 localRawPos = new Vector3(point.point_3d[0], point.point_3d[1], point.point_3d[2]);

        // 2. Apply scaling and Y-locking settings with one-directional Z-scaling
        Vector3 scaledLocalPos = new Vector3(
            localRawPos.x * trajectoryXYScale,
            lockYAxis ? fixedYPosition : localRawPos.y * trajectoryXYScale,
            // Use Mathf.Max(0, ...) to prevent negative (backward) scaling on the Z-axis
            Mathf.Max(0, localRawPos.z) * trajectoryZScale
        );

        // 3. Apply the yaw rotation to the scaled local position
        Vector3 rotatedLocalPos = yawRotation * scaledLocalPos;
        
        // 4. Apply Y-lock after rotation if needed
        if (lockYAxis) {
            rotatedLocalPos.y = fixedYPosition;
        }

        // 5. Transform the rotated local point into world space
        return cameraWorldRotation * rotatedLocalPos + cameraWorldPosition;
    }
    
    void CreateTrailPoint(Vector3 position) {
        GameObject trailPoint = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        trailPoint.name = "TrailPoint";
        trailPoint.transform.position = position;
        trailPoint.transform.localScale = Vector3.one * trailPointSize;
        
        Renderer trailRenderer = trailPoint.GetComponent<Renderer>();
        if (useWallMaterialForGhost && ghostlyRoomMaterial != null) {
            trailRenderer.material = ghostlyRoomMaterial;
        } else {
            trailRenderer.material.color = trailColor;
        }
        
        Destroy(trailPoint.GetComponent<Collider>());
        trailPoints.Add(trailPoint);
    }
    
    void ClearTrailPoints() {
        foreach (GameObject point in trailPoints) {
            if (point != null) {
                Destroy(point); // Use Destroy for runtime objects
            }
        }
        trailPoints.Clear();
    }
    
    // Update trail points to only show up to the specified time
    void UpdateTrailPointsToTime(float time) {
        if (loadedTrajectoryPoints == null || loadedTrajectoryPoints.Count == 0) return;
        if (!createTrailPoints) return;
        
        // Calculate how many frames we should show
        int targetFrame = Mathf.FloorToInt(time * sourceVideoFPS);
        targetFrame = Mathf.Clamp(targetFrame, 0, loadedTrajectoryPoints.Count - 1);
        
        // Clear existing trail points
        ClearTrailPoints();
        
        // Create the rotation from our slider value
        Quaternion yawRotation = Quaternion.Euler(0, trajectoryYawRotation, 0);
        
        // Create trail points up to the target frame
        for (int i = 0; i <= targetFrame; i++) {
            var point = loadedTrajectoryPoints[i];
            if (point.point_3d != null && point.point_3d.Count >= 3) {
                Vector3 worldPos = CalculateWorldPosition(point, yawRotation);
                CreateTrailPoint(worldPos);
            }
        }
        
    }
    
    // Make an object look ghostly with transparency and color
    void MakeObjectGhostly(GameObject obj) {
        Renderer[] renderers = obj.GetComponentsInChildren<Renderer>();
        foreach (Renderer renderer in renderers) {
            if (useWallMaterialForGhost && ghostlyRoomMaterial != null) {
                // Use the same material as the walls
                renderer.material = ghostlyRoomMaterial;
            } else if (ghostMaterial != null) {
                // Use the assigned ghost material
                renderer.material = ghostMaterial;
            }
        }
        
        // Add falling particles if enabled
        if (enableFallingParticles) {
            SetupFallingParticles(obj);
        }
    }
    
    // Create and configure the falling particle system
    void SetupFallingParticles(GameObject ghost) {
        // Create particle system if not assigned
        if (fallingParticles == null) {
            GameObject particleObj = new GameObject("FallingParticles");
            particleObj.transform.SetParent(ghost.transform);
            particleObj.transform.localPosition = Vector3.zero;
            fallingParticles = particleObj.AddComponent<ParticleSystem>();
        } else {
            // Move existing particle system to ghost
            fallingParticles.transform.SetParent(ghost.transform);
            fallingParticles.transform.localPosition = Vector3.zero;
        }
        
        // Configure particle system
        var main = fallingParticles.main;
        main.startLifetime = 3.0f;
        main.startSpeed = 0.0f;
        main.startSize = particleSize;
        main.startColor = particleColor;
        main.maxParticles = 1000;
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        
        // Set material to ensure proper color rendering and round particles
        var renderer = fallingParticles.GetComponent<ParticleSystemRenderer>();
        renderer.material = new Material(Shader.Find("Universal Render Pipeline/Particles/Unlit"));
        renderer.material.color = particleColor;
        renderer.renderMode = ParticleSystemRenderMode.Billboard;
        
        // Emission settings
        var emission = fallingParticles.emission;
        emission.enabled = true;
        emission.rateOverTime = particlesPerSecond;
        
        // Shape settings - emit from ghost's surface
        var shape = fallingParticles.shape;
        shape.enabled = true;
        shape.shapeType = ParticleSystemShapeType.MeshRenderer;
        shape.meshRenderer = ghost.GetComponentInChildren<MeshRenderer>();
        shape.useMeshMaterialIndex = false;
        shape.useMeshColors = false;
        
        // Velocity over lifetime - falling down
        var velocityOverLifetime = fallingParticles.velocityOverLifetime;
        velocityOverLifetime.enabled = true;
        velocityOverLifetime.space = ParticleSystemSimulationSpace.World;
        velocityOverLifetime.y = -particleFallSpeed;
        
        // Color over lifetime - fade out
        var colorOverLifetime = fallingParticles.colorOverLifetime;
        colorOverLifetime.enabled = true;
        Gradient gradient = new Gradient();
        gradient.SetKeys(
            new GradientColorKey[] { new GradientColorKey(particleColor, 0.0f), new GradientColorKey(particleColor, 1.0f) },
            new GradientAlphaKey[] { new GradientAlphaKey(1.0f, 0.0f), new GradientAlphaKey(0.0f, 1.0f) }
        );
        colorOverLifetime.color = gradient;
        
        // Size over lifetime - shrink
        var sizeOverLifetime = fallingParticles.sizeOverLifetime;
        sizeOverLifetime.enabled = true;
        AnimationCurve sizeCurve = new AnimationCurve();
        sizeCurve.AddKey(0.0f, 1.0f);
        sizeCurve.AddKey(1.0f, 0.0f);
        sizeOverLifetime.size = new ParticleSystem.MinMaxCurve(1.0f, sizeCurve);
        
        // Start the particle system
        fallingParticles.Play();
        
    }
    
    // Setup UI controls
    void SetupUI() {
        // Create Canvas if not assigned
        if (uiCanvas == null) {
            GameObject canvasObj = new GameObject("UI Canvas");
            uiCanvas = canvasObj.AddComponent<Canvas>();
            uiCanvas.renderMode = RenderMode.ScreenSpaceCamera; // Use camera space to get post-processing effects
            uiCanvas.worldCamera = mainCamera; // Assign main camera
            uiCanvas.planeDistance = 1.0f; // Further from camera to ensure UI is visible
            uiCanvas.sortingOrder = 100; // Higher order to ensure UI is visible
            
            // Add CanvasScaler for proper scaling
            CanvasScaler scaler = canvasObj.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);
            scaler.screenMatchMode = CanvasScaler.ScreenMatchMode.MatchWidthOrHeight;
            scaler.matchWidthOrHeight = 0.5f;
            
            // Add GraphicRaycaster for UI interaction
            canvasObj.AddComponent<GraphicRaycaster>();
            
            // Add EventSystem if it doesn't exist
            if (FindObjectOfType<UnityEngine.EventSystems.EventSystem>() == null) {
                GameObject eventSystemObj = new GameObject("EventSystem");
                eventSystemObj.AddComponent<UnityEngine.EventSystems.EventSystem>();
                eventSystemObj.AddComponent<UnityEngine.EventSystems.StandaloneInputModule>();
            }
        }
        
        // Create simple button instead of toggle
        if (pointCloudToggle == null) {
            // Create button
            GameObject buttonObj = new GameObject("Point Cloud Button");
            buttonObj.transform.SetParent(uiCanvas.transform, false);
            
            // Add Image component for background
            Image buttonImage = buttonObj.AddComponent<Image>();
            buttonImage.color = new Color(0, 0, 0, 0f);
            
            // Set position in top-right corner
            RectTransform buttonRect = buttonObj.GetComponent<RectTransform>();
            buttonRect.anchorMin = new Vector2(1, 1);
            buttonRect.anchorMax = new Vector2(1, 1);
            buttonRect.pivot = new Vector2(1, 1);
            buttonRect.anchoredPosition = new Vector2(-20, -20);
            buttonRect.sizeDelta = new Vector2(250, 60);
            
            // Add Button component
            Button button = buttonObj.AddComponent<Button>();
            button.targetGraphic = buttonImage;
            
            // Create label
            GameObject labelObj = new GameObject("Label");
            labelObj.transform.SetParent(buttonObj.transform, false);
            Text labelText = labelObj.AddComponent<Text>();
            labelText.text = usePointCloud ? "Switch to Solid" : "Switch to Point Cloud";
            labelText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            labelText.fontSize = 18;
            labelText.color = Color.white;
            labelText.alignment = TextAnchor.MiddleCenter;
            
            RectTransform labelRect = labelObj.GetComponent<RectTransform>();
            labelRect.anchorMin = Vector2.zero;
            labelRect.anchorMax = Vector2.one;
            labelRect.offsetMin = Vector2.zero;
            labelRect.offsetMax = Vector2.zero;
            
            // Add click listener
            button.onClick.AddListener(() => {
                // Store current camera orbit state before switching
                Vector3 currentCameraPosition = mainCamera.transform.position;
                Vector3 currentOrbitCenter = orbitCenter;
                float currentOrbitDistance = Vector3.Distance(currentCameraPosition, currentOrbitCenter);
                
                // Calculate current orbit angles to preserve them
                Vector3 direction = (currentCameraPosition - currentOrbitCenter).normalized;
                float currentOrbitX = Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;
                float currentOrbitY = Mathf.Asin(direction.y) * Mathf.Rad2Deg;
                
                usePointCloud = !usePointCloud;
                labelText.text = usePointCloud ? "Switch to Solid" : "Switch to Point Cloud";
                LoadRoomModel(); // Reload the room model with new setting
                ChangeSkybox(usePointCloud ? pointCloudSkybox : solidModelSkybox); // Switch skybox too
                
                // Restore camera orbit state after room model is loaded
                StartCoroutine(RestoreCameraOrbitState(currentOrbitX, currentOrbitY, currentOrbitDistance));
                
            });
        }
        
    }
    
    // Setup playback control UI
    void SetupPlaybackControls() {
        if (uiCanvas == null) return;
        
        // Create playback panel
        GameObject playbackPanel = new GameObject("Playback Panel");
        playbackPanel.transform.SetParent(uiCanvas.transform, false);
        
        // Add background image (transparent)
        Image panelImage = playbackPanel.AddComponent<Image>();
        panelImage.color = new Color(0, 0, 0, 0f);
        
        // Position at bottom of screen
        RectTransform panelRect = playbackPanel.GetComponent<RectTransform>();
        panelRect.anchorMin = new Vector2(0, 0);
        panelRect.anchorMax = new Vector2(1, 0);
        panelRect.pivot = new Vector2(0.5f, 0);
        panelRect.anchoredPosition = new Vector2(0, 0);
        panelRect.sizeDelta = new Vector2(0, playbackPanelHeight);
        
        // Create play/pause button
        CreatePlayButton(playbackPanel);
        
        // Create stop button
        CreateStopButton(playbackPanel);
        
        // Create timeline slider
        CreateTimelineSlider(playbackPanel);
        
        // Create time display
        CreateTimeDisplay(playbackPanel);
        
    }
    
    // Setup audio system
    void SetupAudio() {
        // Create background music audio source
        GameObject bgMusicObj = new GameObject("Background Music");
        bgMusicObj.transform.SetParent(transform);
        backgroundMusicSource = bgMusicObj.AddComponent<AudioSource>();
        backgroundMusicSource.clip = backgroundMusic;
        backgroundMusicSource.loop = true;
        backgroundMusicSource.volume = backgroundMusicVolume;
        backgroundMusicSource.playOnAwake = true;
        originalBackgroundVolume = backgroundMusicVolume;
        
        // Start background music
        if (backgroundMusic != null) {
            backgroundMusicSource.Play();
        } else {
            Debug.LogWarning("Background music clip not assigned!");
        }
        
        // Create shimmer audio source (will be attached to ghost when created)
        GameObject shimmerObj = new GameObject("Shimmer Audio");
        shimmerObj.transform.SetParent(transform);
        shimmerAudioSource = shimmerObj.AddComponent<AudioSource>();
        shimmerAudioSource.clip = shimmerSound;
        shimmerAudioSource.loop = true;
        shimmerAudioSource.volume = 0f; // Start silent
        shimmerAudioSource.playOnAwake = true;
        shimmerAudioSource.spatialBlend = 1f; // 3D sound
        shimmerAudioSource.rolloffMode = AudioRolloffMode.Linear;
        shimmerAudioSource.minDistance = shimmerMaxDistance;
        shimmerAudioSource.maxDistance = shimmerMinDistance;
        
        // Start shimmer sound (silent initially)
        if (shimmerSound != null) {
            shimmerAudioSource.Play();
        } else {
            Debug.LogWarning("Shimmer sound clip not assigned!");
        }
        
    }
    
    // Update audio volumes based on distance to ghost
    void UpdateAudioBasedOnDistance() {
        if (currentPersonInstance == null || mainCamera == null || shimmerAudioSource == null || backgroundMusicSource == null) return;
        
        // Pause shimmer if animation is stopped or scrubbing
        if (!isPlaying || isScrubbing) {
            if (shimmerAudioSource.isPlaying) {
                shimmerAudioSource.Pause();
            }
            // Reset background music to full volume when shimmer is paused
            backgroundMusicSource.volume = originalBackgroundVolume;
            return;
        }
        
        // Resume shimmer if animation is playing and not scrubbing
        if (isPlaying && !isPaused && !isScrubbing && !shimmerAudioSource.isPlaying) {
            shimmerAudioSource.UnPause();
        }
        
        // Calculate distance from camera to ghost
        float distance = Vector3.Distance(mainCamera.transform.position, currentPersonInstance.transform.position);
        
        // Update shimmer audio source position to follow ghost
        shimmerAudioSource.transform.position = currentPersonInstance.transform.position;
        
        // Calculate shimmer volume based on distance
        float shimmerVolumeMultiplier = 0f;
        if (distance <= shimmerMaxDistance) {
            shimmerVolumeMultiplier = 1f; // Maximum volume when very close
        } else if (distance <= shimmerMinDistance) {
            // Fade in as you get closer
            shimmerVolumeMultiplier = 1f - ((distance - shimmerMaxDistance) / (shimmerMinDistance - shimmerMaxDistance));
        }
        
        shimmerAudioSource.volume = shimmerVolume * shimmerVolumeMultiplier;
        
        // Duck background music when close to ghost
        float backgroundDucking = 1f;
        if (distance <= shimmerMinDistance) {
            backgroundDucking = 0.7f + (0.3f * ((distance - shimmerMaxDistance) / (shimmerMinDistance - shimmerMaxDistance)));
            backgroundDucking = Mathf.Clamp(backgroundDucking, 0.7f, 1f);
        }
        
        backgroundMusicSource.volume = originalBackgroundVolume * backgroundDucking;
    }
    
    void CreatePlayButton(GameObject parent) {
        GameObject playBtn = new GameObject("Play Button");
        playBtn.transform.SetParent(parent.transform, false);
        
        Image btnImage = playBtn.AddComponent<Image>();
        btnImage.color = new Color(0, 0, 0, 0f);
        
        RectTransform btnRect = playBtn.GetComponent<RectTransform>();
        btnRect.anchorMin = new Vector2(0, 0.5f);
        btnRect.anchorMax = new Vector2(0, 0.5f);
        btnRect.pivot = new Vector2(0, 0.5f);
        btnRect.anchoredPosition = new Vector2(80, 0);
        btnRect.sizeDelta = new Vector2(50, 50); // Square button
        
        playButton = playBtn.AddComponent<Button>();
        playButton.targetGraphic = btnImage;
        
        CreatePauseIcon(playBtn);
        
        playButton.onClick.AddListener(TogglePlayPause);
    }
    
    void CreatePauseIcon(GameObject parent) {
        // Clear existing icons first
        foreach (Transform child in parent.transform) {
            if (child.name.Contains("Line") || child.name.Contains("Triangle")) {
                Destroy(child.gameObject);
            }
        }
        
        // Left line
        GameObject leftLine = new GameObject("Left Line");
        leftLine.transform.SetParent(parent.transform, false);
        Image leftImage = leftLine.AddComponent<Image>();
        leftImage.color = Color.white;
        
        RectTransform leftRect = leftLine.GetComponent<RectTransform>();
        leftRect.anchorMin = new Vector2(0.35f, 0.3f);
        leftRect.anchorMax = new Vector2(0.42f, 0.7f);
        leftRect.offsetMin = Vector2.zero;
        leftRect.offsetMax = Vector2.zero;
        
        // Right line
        GameObject rightLine = new GameObject("Right Line");
        rightLine.transform.SetParent(parent.transform, false);
        Image rightImage = rightLine.AddComponent<Image>();
        rightImage.color = Color.white;
        
        RectTransform rightRect = rightLine.GetComponent<RectTransform>();
        rightRect.anchorMin = new Vector2(0.58f, 0.3f);
        rightRect.anchorMax = new Vector2(0.65f, 0.7f);
        rightRect.offsetMin = Vector2.zero;
        rightRect.offsetMax = Vector2.zero;
    }
    
    void CreatePlayIcon(GameObject parent) {
        // Clear existing icons
        foreach (Transform child in parent.transform) {
            if (child.name.Contains("Line") || child.name.Contains("Triangle")) {
                Destroy(child.gameObject);
            }
        }
        
        // Create triangle pointing right using Text
        GameObject triangle = new GameObject("Triangle");
        triangle.transform.SetParent(parent.transform, false);
        
        Text triangleText = triangle.AddComponent<Text>();
        triangleText.text = "▶";
        triangleText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        triangleText.fontSize = 45; // Even larger triangle
        triangleText.color = Color.white;
        triangleText.alignment = TextAnchor.MiddleCenter;
        
        RectTransform triangleRect = triangle.GetComponent<RectTransform>();
        triangleRect.anchorMin = Vector2.zero;
        triangleRect.anchorMax = Vector2.one;
        triangleRect.offsetMin = Vector2.zero;
        triangleRect.offsetMax = Vector2.zero;
    }
    
    void CreateStopButton(GameObject parent) {
        GameObject stopBtn = new GameObject("Stop Button");
        stopBtn.transform.SetParent(parent.transform, false);
        
        Image btnImage = stopBtn.AddComponent<Image>();
        btnImage.color = new Color(0, 0, 0, 0f);
        
        RectTransform btnRect = stopBtn.GetComponent<RectTransform>();
        btnRect.anchorMin = new Vector2(0, 0.5f);
        btnRect.anchorMax = new Vector2(0, 0.5f);
        btnRect.pivot = new Vector2(0, 0.5f);
        btnRect.anchoredPosition = new Vector2(20, 0);
        btnRect.sizeDelta = new Vector2(35, 35);
        
        stopButton = stopBtn.AddComponent<Button>();
        stopButton.targetGraphic = btnImage;
        
        // Create white square icon
        GameObject square = new GameObject("Square");
        square.transform.SetParent(stopBtn.transform, false);
        Image squareImage = square.AddComponent<Image>();
        squareImage.color = Color.white;
        
        RectTransform squareRect = square.GetComponent<RectTransform>();
        squareRect.anchorMin = new Vector2(0.3f, 0.3f);
        squareRect.anchorMax = new Vector2(0.7f, 0.7f);
        squareRect.offsetMin = Vector2.zero;
        squareRect.offsetMax = Vector2.zero;
        
        stopButton.onClick.AddListener(StopAnimation);
    }
    
    void CreateTimelineSlider(GameObject parent) {
        GameObject sliderObj = new GameObject("Timeline Slider");
        sliderObj.transform.SetParent(parent.transform, false);
        
        RectTransform sliderRect = sliderObj.AddComponent<RectTransform>();
        sliderRect.anchorMin = new Vector2(0, 0.5f);
        sliderRect.anchorMax = new Vector2(1, 0.5f);
        sliderRect.pivot = new Vector2(0.5f, 0.5f);
        sliderRect.anchoredPosition = new Vector2(0, 0);
        sliderRect.offsetMin = new Vector2(140, -6);
        sliderRect.offsetMax = new Vector2(-100, 6);
        
        timelineSlider = sliderObj.AddComponent<Slider>();
        timelineSlider.minValue = 0f;
        timelineSlider.maxValue = 1f;
        timelineSlider.value = 0f;
        
        // Add background
        GameObject background = new GameObject("Background");
        background.transform.SetParent(sliderObj.transform, false);
        Image bgImage = background.AddComponent<Image>();
        bgImage.color = new Color(0.2f, 0.2f, 0.2f, 1f);
        
        RectTransform bgRect = background.GetComponent<RectTransform>();
        bgRect.anchorMin = Vector2.zero;
        bgRect.anchorMax = Vector2.one;
        bgRect.offsetMin = Vector2.zero;
        bgRect.offsetMax = Vector2.zero;
        
        // Add fill area
        GameObject fillArea = new GameObject("Fill Area");
        fillArea.transform.SetParent(sliderObj.transform, false);
        RectTransform fillAreaRect = fillArea.AddComponent<RectTransform>();
        fillAreaRect.anchorMin = Vector2.zero;
        fillAreaRect.anchorMax = Vector2.one;
        fillAreaRect.offsetMin = Vector2.zero;
        fillAreaRect.offsetMax = Vector2.zero;
        
        // Add fill
        GameObject fill = new GameObject("Fill");
        fill.transform.SetParent(fillArea.transform, false);
        Image fillImage = fill.AddComponent<Image>();
        fillImage.color = new Color(1f, 1f, 1f, 0.5f);
        
        RectTransform fillRect = fill.GetComponent<RectTransform>();
        fillRect.anchorMin = Vector2.zero;
        fillRect.anchorMax = Vector2.one;
        fillRect.offsetMin = Vector2.zero;
        fillRect.offsetMax = Vector2.zero;
        
        timelineSlider.fillRect = fillRect;
        
        // Add handle
        GameObject handleArea = new GameObject("Handle Slide Area");
        handleArea.transform.SetParent(sliderObj.transform, false);
        RectTransform handleAreaRect = handleArea.AddComponent<RectTransform>();
        handleAreaRect.anchorMin = Vector2.zero;
        handleAreaRect.anchorMax = Vector2.one;
        handleAreaRect.offsetMin = Vector2.zero;
        handleAreaRect.offsetMax = Vector2.zero;
        
        GameObject handle = new GameObject("Handle");
        handle.transform.SetParent(handleArea.transform, false);
        Image handleImage = handle.AddComponent<Image>();
        handleImage.color = Color.white;
        
        RectTransform handleRect = handle.GetComponent<RectTransform>();
        handleRect.anchorMin = new Vector2(0.5f, 0.5f);
        handleRect.anchorMax = new Vector2(0.5f, 0.5f);
        handleRect.pivot = new Vector2(0.5f, 0.5f);
        handleRect.anchoredPosition = Vector2.zero;
        handleRect.sizeDelta = new Vector2(12, 20);
        
        timelineSlider.handleRect = handleRect;
        
        timelineSlider.onValueChanged.AddListener(OnTimelineScrub);
        
        // Add event triggers for mouse down and up to detect scrubbing
        var eventTrigger = timelineSlider.gameObject.AddComponent<UnityEngine.EventSystems.EventTrigger>();
        
        // Mouse down - start scrubbing
        var pointerDownEntry = new UnityEngine.EventSystems.EventTrigger.Entry();
        pointerDownEntry.eventID = UnityEngine.EventSystems.EventTriggerType.PointerDown;
        pointerDownEntry.callback.AddListener((data) => { OnTimelineScrubStart(); });
        eventTrigger.triggers.Add(pointerDownEntry);
        
        // Mouse up - end scrubbing
        var pointerUpEntry = new UnityEngine.EventSystems.EventTrigger.Entry();
        pointerUpEntry.eventID = UnityEngine.EventSystems.EventTriggerType.PointerUp;
        pointerUpEntry.callback.AddListener((data) => { OnTimelineScrubEnd(); });
        eventTrigger.triggers.Add(pointerUpEntry);
        
        // Click anywhere on timeline to jump to that position
        var pointerClickEntry = new UnityEngine.EventSystems.EventTrigger.Entry();
        pointerClickEntry.eventID = UnityEngine.EventSystems.EventTriggerType.PointerClick;
        pointerClickEntry.callback.AddListener((data) => { OnTimelineClick(data); });
        eventTrigger.triggers.Add(pointerClickEntry);
    }
    
    void CreateTimeDisplay(GameObject parent) {
        GameObject timeObj = new GameObject("Time Display");
        timeObj.transform.SetParent(parent.transform, false);
        
        timeText = timeObj.AddComponent<Text>();
        timeText.text = "0:00 / 0:00";
        timeText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        timeText.fontSize = 14;
        timeText.color = Color.white;
        timeText.alignment = TextAnchor.MiddleRight;
        
        RectTransform timeRect = timeObj.GetComponent<RectTransform>();
        timeRect.anchorMin = new Vector2(1, 0.5f);
        timeRect.anchorMax = new Vector2(1, 0.5f);
        timeRect.pivot = new Vector2(1, 0.5f);
        timeRect.anchoredPosition = new Vector2(-20, 0);
        timeRect.sizeDelta = new Vector2(80, 30);
    }
    
    // Calculate the center of the room model for orbit camera
    void CalculateRoomCenter(GameObject roomModel) {
        if (roomModel == null) return;
        
        // Get all mesh renderers in the room model
        MeshRenderer[] renderers = roomModel.GetComponentsInChildren<MeshRenderer>();
        if (renderers.Length == 0) return;
        
        // Calculate the combined bounds of all meshes
        Bounds combinedBounds = renderers[0].bounds;
        for (int i = 1; i < renderers.Length; i++) {
            combinedBounds.Encapsulate(renderers[i].bounds);
        }
        
        // Set orbit center to the center of the room
        orbitCenter = combinedBounds.center;
        
        // Set a good default distance based on room size
        float roomSize = Mathf.Max(combinedBounds.size.x, combinedBounds.size.y, combinedBounds.size.z);
        orbitDistance = roomSize * 1.5f;
        
    }
    
    // Handle orbit camera controls
    void HandleOrbitCamera() {
        // Handle mouse input
        if (Input.GetMouseButtonDown(0)) {
            isDragging = true;
        }
        
        if (Input.GetMouseButtonUp(0)) {
            isDragging = false;
        }
        
        // Rotate camera around orbit center
        if (isDragging) {
            // Use much higher multipliers for more responsive control
            float mouseX = Input.GetAxis("Mouse X") * mouseSensitivity * 2.5f;
            float mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity * 2.5f;
            
            orbitX += mouseX;
            orbitY -= mouseY; // Invert Y for natural feel
            
            // Clamp vertical rotation to prevent flipping
            orbitY = Mathf.Clamp(orbitY, -80f, 80f);
        }
        
        // Handle zoom with scroll wheel
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (scroll != 0f) {
            orbitDistance -= scroll * zoomSensitivity;
            orbitDistance = Mathf.Clamp(orbitDistance, minDistance, maxDistance);
        }
        
        // Calculate camera position based on orbit angles and distance
        float radX = orbitX * Mathf.Deg2Rad;
        float radY = orbitY * Mathf.Deg2Rad;
        
        Vector3 offset = new Vector3(
            Mathf.Sin(radX) * Mathf.Cos(radY) * orbitDistance,
            Mathf.Sin(radY) * orbitDistance,
            Mathf.Cos(radX) * Mathf.Cos(radY) * orbitDistance
        );
        
        // Set camera position and look at orbit center
        mainCamera.transform.position = orbitCenter + offset;
        mainCamera.transform.LookAt(orbitCenter);
    }
    
    // Handle playback controls update
    void HandlePlaybackControls() {
        if (loadedTrajectoryPoints == null || loadedTrajectoryPoints.Count == 0) return;
        
        // Calculate total duration
        if (totalDuration == 0f) {
            totalDuration = loadedTrajectoryPoints.Count / sourceVideoFPS;
        }
        
        // Update current time if playing (but don't auto-advance if we have manual control or are scrubbing)
        if (isPlaying && !isPaused && !isScrubbing && animationCoroutine == null) {
            currentTime += Time.deltaTime * playbackSpeed;
            if (currentTime >= totalDuration) {
                currentTime = totalDuration;
                isPlaying = false;
            }
        }
        
        // Update timeline slider
        if (timelineSlider != null) {
            timelineSlider.value = currentTime / totalDuration;
        }
        
        // Update time display
        if (timeText != null) {
            string currentTimeStr = FormatTime(currentTime);
            string totalTimeStr = FormatTime(totalDuration);
            timeText.text = $"{currentTimeStr} / {totalTimeStr}";
        }
        
        // Update play button icon only when state changes
        if (playButton != null) {
            bool currentPlayState = isPlaying && !isPaused;
            if (currentPlayState != lastPlayState) {
                if (currentPlayState) {
                    // Show pause icon (two lines)
                    CreatePauseIcon(playButton.gameObject);
                } else {
                    // Show play icon (triangle)
                    CreatePlayIcon(playButton.gameObject);
                }
                lastPlayState = currentPlayState;
            }
        }
    }
    
    // Format time as MM:SS
    string FormatTime(float time) {
        int minutes = Mathf.FloorToInt(time / 60f);
        int seconds = Mathf.FloorToInt(time % 60f);
        return $"{minutes}:{seconds:00}";
    }
    
    // Toggle play/pause
    void TogglePlayPause() {
        if (isPlaying && !isPaused) {
            isPaused = true;
            // Pause the animation coroutine
            if (animationCoroutine != null) {
                StopCoroutine(animationCoroutine);
                animationCoroutine = null;
            }
        } else {
            isPlaying = true;
            isPaused = false;
            // Start animation from current time position
            if (animationCoroutine == null) {
                StartAnimationFromCurrentTime();
            }
        }
    }
    
    // Stop animation
    void StopAnimation() {
        isPlaying = false;
        isPaused = false;
        currentTime = 0f;
        
        // Stop the animation coroutine
        if (animationCoroutine != null) {
            StopCoroutine(animationCoroutine);
            animationCoroutine = null;
        }
        
        // Reset ghost to first position
        if (currentPersonInstance != null && loadedTrajectoryPoints != null && loadedTrajectoryPoints.Count > 0) {
            var firstPoint = loadedTrajectoryPoints[0];
            if (firstPoint.point_3d != null && firstPoint.point_3d.Count >= 3) {
                Vector3 firstPos = CalculateWorldPosition(firstPoint, Quaternion.Euler(0, trajectoryYawRotation, 0));
                currentPersonInstance.transform.position = firstPos;
            }
        }
        
        // Clear all trail points when stopping
        ClearTrailPoints();
        
        if (timelineSlider != null) {
            timelineSlider.value = 0f;
        }
    }
    
    // Handle timeline scrubbing
    void OnTimelineScrub(float value) {
        if (loadedTrajectoryPoints == null || loadedTrajectoryPoints.Count == 0) return;
        
        currentTime = value * totalDuration;
        
        // Jump to the corresponding frame in the animation
        int targetFrame = Mathf.FloorToInt(currentTime * sourceVideoFPS);
        targetFrame = Mathf.Clamp(targetFrame, 0, loadedTrajectoryPoints.Count - 1);
        
        // Update ghost position to the target frame
        if (currentPersonInstance != null && targetFrame < loadedTrajectoryPoints.Count) {
            var targetPoint = loadedTrajectoryPoints[targetFrame];
            if (targetPoint.point_3d != null && targetPoint.point_3d.Count >= 3) {
                Vector3 targetPos = CalculateWorldPosition(targetPoint, Quaternion.Euler(0, trajectoryYawRotation, 0));
                currentPersonInstance.transform.position = targetPos;
            }
        }
        
        // Update trail points to only show up to current time
        UpdateTrailPointsToTime(currentTime);
    }
    
    // Called when user starts scrubbing the timeline
    void OnTimelineScrubStart() {
        if (isScrubbing) return; // Already scrubbing
        
        isScrubbing = true;
        wasPlayingBeforeScrub = isPlaying && !isPaused;
        
        // Pause the animation if it was playing
        if (wasPlayingBeforeScrub) {
            isPaused = true;
            if (animationCoroutine != null) {
                StopCoroutine(animationCoroutine);
                animationCoroutine = null;
            }
        }
    }
    
    // Called when user stops scrubbing the timeline
    void OnTimelineScrubEnd() {
        if (!isScrubbing) return; // Not scrubbing
        
        isScrubbing = false;
        
        // Resume playing if we were playing before scrubbing
        if (wasPlayingBeforeScrub) {
            isPaused = false;
            isPlaying = true;
            
            // Start animation from current scrubbed position
            if (animationCoroutine == null) {
                StartAnimationFromCurrentTime();
            }
        }
        
        wasPlayingBeforeScrub = false;
    }
    
    // Called when user clicks anywhere on the timeline to jump to that position
    void OnTimelineClick(UnityEngine.EventSystems.BaseEventData data) {
        if (timelineSlider == null || loadedTrajectoryPoints == null || loadedTrajectoryPoints.Count == 0) return;
        
        // Get the click position relative to the slider
        var pointerData = data as UnityEngine.EventSystems.PointerEventData;
        if (pointerData == null) return;
        
        RectTransform sliderRect = timelineSlider.GetComponent<RectTransform>();
        Vector2 localPoint;
        RectTransformUtility.ScreenPointToLocalPointInRectangle(sliderRect, pointerData.position, pointerData.pressEventCamera, out localPoint);
        
        // Calculate the normalized position (0 to 1) based on click position
        float normalizedPosition = (localPoint.x + sliderRect.rect.width * 0.5f) / sliderRect.rect.width;
        normalizedPosition = Mathf.Clamp01(normalizedPosition);
        
        // Set the slider value and update the animation
        timelineSlider.value = normalizedPosition;
        OnTimelineScrub(normalizedPosition);
        
    }
    
    // Start animation from current time position
    void StartAnimationFromCurrentTime() {
        if (loadedTrajectoryPoints == null || loadedTrajectoryPoints.Count == 0) return;
        
        // Calculate which frame we should start from
        int startFrame = Mathf.FloorToInt(currentTime * sourceVideoFPS);
        startFrame = Mathf.Clamp(startFrame, 0, loadedTrajectoryPoints.Count - 1);
        
        // Start animation from the current frame
        animationCoroutine = StartCoroutine(AnimateTrajectoryFromFrame(loadedTrajectoryPoints, startFrame));
    }
    
    // Modified animation coroutine that starts from a specific frame
    IEnumerator AnimateTrajectoryFromFrame(List<RawTrajectoryPoint> points, int startFrame) {
        if (personPrefab == null) {
            Debug.LogError("Person Prefab is not assigned!"); 
            yield break;
        }

        // Create the rotation from our slider value
        Quaternion yawRotation = Quaternion.Euler(0, trajectoryYawRotation, 0);
        
        // Create the ghost instance if it doesn't exist
        if (currentPersonInstance == null) {
            // Find first valid point (skip null point_3d entries)
            int validIndex = 0;
            while (validIndex < points.Count && (points[validIndex].point_3d == null || points[validIndex].point_3d.Count < 3)) {
                validIndex++;
            }
            
            if (validIndex >= points.Count) {
                Debug.LogError("No valid 3D trajectory points found!");
                yield break;
            }
            
            // Calculate initial position for the ghost
            Vector3 initialLocalPos = new Vector3(points[validIndex].point_3d[0], points[validIndex].point_3d[1], points[validIndex].point_3d[2]);
            Vector3 scaledInitialLocalPos = new Vector3(
                initialLocalPos.x * trajectoryXYScale,
                lockYAxis ? fixedYPosition : initialLocalPos.y * trajectoryXYScale,
                Mathf.Max(0, initialLocalPos.z) * trajectoryZScale
            );
            
            Vector3 rotatedInitialLocalPos = yawRotation * scaledInitialLocalPos;
            Vector3 initialWorldPos = cameraWorldRotation * rotatedInitialLocalPos + cameraWorldPosition;
            
            currentPersonInstance = Instantiate(personPrefab, initialWorldPos, Quaternion.identity);
            currentPersonInstance.transform.localScale = Vector3.one * personScale;
            Camera[] ghostCameras = currentPersonInstance.GetComponentsInChildren<Camera>();
            foreach (Camera cam in ghostCameras) { cam.enabled = false; }
            if (makePersonGhostly) { MakeObjectGhostly(currentPersonInstance); }
            
            // Attach shimmer audio source to ghost
            if (shimmerAudioSource != null) {
                shimmerAudioSource.transform.SetParent(currentPersonInstance.transform);
                shimmerAudioSource.transform.localPosition = Vector3.zero;
            }
            
        }
        

        // Process trajectory points starting from the specified frame
        for (int i = startFrame; i < points.Count - 1; i++) {
            // Check if we should pause
            if (isPaused || !isPlaying) {
                yield break;
            }
            
            var currentPoint = points[i];
            var nextPoint = points[i + 1];
            
            if (currentPoint.point_3d == null || currentPoint.point_3d.Count < 3) continue;
            if (nextPoint.point_3d == null || nextPoint.point_3d.Count < 3) continue;

            // Calculate the start and end positions for this segment
            Vector3 startPos = CalculateWorldPosition(currentPoint, yawRotation);
            Vector3 endPos = CalculateWorldPosition(nextPoint, yawRotation);
            
            // Interpolate between start and end positions
            for (int step = 0; step < interpolationSteps; step++) {
                // Check if we should pause during interpolation
                if (isPaused || !isPlaying) {
                    yield break;
                }
                
                float t = (float)step / interpolationSteps;
                t = t * t * (3f - 2f * t); // Smooth interpolation
                
                Vector3 interpolatedPos = Vector3.Lerp(startPos, endPos, t);
                
                // Apply hover effect
                Vector3 finalGhostPosition = interpolatedPos;
                if (enableHover) {
                    float hoverOffset = Mathf.Sin(Time.time * hoverSpeed) * hoverAmplitude;
                    finalGhostPosition += new Vector3(0, hoverOffset, 0);
                }
                currentPersonInstance.transform.position = finalGhostPosition;
                
                // Apply spin effect
                if (enableSpin) {
                    currentPersonInstance.transform.Rotate(0, spinSpeed * Time.deltaTime, 0, Space.World);
                }
                
                // Update current time based on frame progress
                currentTime = (i + (float)step / interpolationSteps) / sourceVideoFPS;
                
                // Update trail points to show progress up to current time
                UpdateTrailPointsToTime(currentTime);
                
                yield return new WaitForSeconds((1.0f / sourceVideoFPS) / (playbackSpeed * interpolationSteps));
            }
        }
        
        // Animation finished
        isPlaying = false;
        animationCoroutine = null;
    }
    
    
    
    // Change the skybox material
    void ChangeSkybox(Material skyboxMaterial) {
        if (skyboxMaterial != null) {
            RenderSettings.skybox = skyboxMaterial;
        } else {
            // If no skybox material is assigned, create a black one for point cloud mode
            if (usePointCloud) {
                Material blackSkybox = CreateBlackSkybox();
                RenderSettings.skybox = blackSkybox;
            } else {
                Debug.LogWarning("Skybox material not assigned, cannot change skybox");
            }
        }
    }
    
    // Create a simple black skybox material
    Material CreateBlackSkybox() {
        Material blackSkybox = new Material(Shader.Find("Skybox/6 Sided"));
        blackSkybox.name = "BlackSkybox";
        
        // Set all faces to black
        blackSkybox.SetColor("_Tint", Color.black);
        blackSkybox.SetFloat("_Exposure", 0f);
        
        return blackSkybox;
    }
    
    // Coroutine to restore camera orbit state after room model is loaded
    IEnumerator RestoreCameraOrbitState(float targetOrbitX, float targetOrbitY, float targetOrbitDistance) {
        // Wait one frame to ensure the room model is fully loaded and orbit center is updated
        yield return null;
        
        // Update our orbit variables to match the stored state
        orbitX = targetOrbitX;
        orbitY = targetOrbitY;
        orbitDistance = targetOrbitDistance;
        
        // Calculate the new camera position based on the preserved orbit state
        float radX = orbitX * Mathf.Deg2Rad;
        float radY = orbitY * Mathf.Deg2Rad;
        
        Vector3 offset = new Vector3(
            Mathf.Sin(radX) * Mathf.Cos(radY) * orbitDistance,
            Mathf.Sin(radY) * orbitDistance,
            Mathf.Cos(radX) * Mathf.Cos(radY) * orbitDistance
        );
        
        // Set camera position and look at the new orbit center
        mainCamera.transform.position = orbitCenter + offset;
        mainCamera.transform.LookAt(orbitCenter);
        
    }
    
    // Start the initial animation when playback controls are set up
    IEnumerator StartInitialAnimation() {
        // Wait one frame to ensure everything is set up
        yield return null;
        
        // Start the animation from the beginning
        if (loadedTrajectoryPoints != null && loadedTrajectoryPoints.Count > 0) {
            currentTime = 0f;
            isPlaying = true;
            isPaused = false;
            
            // Stop any existing animation first
            if (animationCoroutine != null) {
                StopCoroutine(animationCoroutine);
                animationCoroutine = null;
            }
            
            // Start the animation coroutine
            animationCoroutine = StartCoroutine(AnimateTrajectoryFromFrame(loadedTrajectoryPoints, 0));
            
        }
    }
    #endregion
    
}