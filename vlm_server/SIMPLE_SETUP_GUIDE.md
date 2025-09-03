# Simple Screen Capture VLM Control Setup

This is a **much simpler** approach that uses desktop screen capture instead of complex ROS/FastAPI integration!

## 🎯 How It Works

1. **Use existing Duckietown GUIs** to display camera feed and control
2. **Capture that screen area** with the VLM system  
3. **Send keyboard commands** to the control GUI
4. **No complex video streaming** or ROS integration needed!

## 📋 Quick Setup (4 Steps)

### Step 1: Start Duckiebot Camera GUI
```bash
# Launch GUI tools for your robot
dts start_gui_tools YOUR_DUCKIEBOT_NAME

# In the container, start camera viewer
rqt_image_view
```

In the `rqt_image_view` window:
- Select `camera_node/image/compressed` from dropdown
- You should see your robot's camera feed

### Step 2: Start Keyboard Control GUI  
```bash
# In another terminal, launch keyboard control
dts duckiebot keyboard_control YOUR_DUCKIEBOT_NAME
```

This opens the keyboard control GUI that responds to arrow keys.

### Step 3: Start llama.cpp Server
```bash
# Start the VLM server
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 --n-gpu-layers 99
```

### Step 4: Run Screen Capture VLM
```bash
cd vlm_server
pip install mss pygetwindow pyautogui keyboard
python simple_screen_capture_vlm.py
```

Follow the setup instructions to select the camera window area.

## 🎮 Usage

### Controls:
- **SPACE**: Toggle auto control ON/OFF
- **T**: Toggle between Arrow keys and WASD
- **M**: Manual single capture/decision
- **Q**: Quit

### Auto Mode:
1. Press SPACE to enable auto control
2. The system captures the camera view every 0.5 seconds
3. VLM analyzes the image and makes decisions
4. Keyboard commands are sent to control the robot

### Manual Mode:
- Press M to capture and analyze a single frame
- Review the decision before enabling auto mode

## 🔧 Configuration

### Adjust Capture Rate:
```bash
python simple_screen_capture_vlm.py --fps 1.0  # Slower: 1 FPS
python simple_screen_capture_vlm.py --fps 3.0  # Faster: 3 FPS
```

### Key Mapping:
The system supports both control schemes:
- **Arrow Keys** (default): ↑↓←→ and SPACE
- **WASD**: w/a/s/d keys (press T to toggle)

## 🏗️ Architecture Comparison

### ❌ Complex Approach (Previous):
```
Duckiebot Camera → ROS → Video Streamer → FastAPI → VLM → Keyboard Simulation
```

### ✅ Simple Approach (This):
```
Duckiebot Camera → GUI Display → Screen Capture → VLM → Keyboard Input
```

## 📊 Benefits

### **Much Simpler**:
- Uses existing Duckietown tools
- No ROS integration complexity
- No FastAPI server needed
- No video streaming protocols

### **More Reliable**:
- Direct keyboard input to existing GUI
- Visual feedback (you see what VLM sees)
- Easy debugging and monitoring

### **Easier Setup**:
- Just 4 terminal commands
- Auto-detection of GUI windows
- Interactive region selection

## 🛠️ Troubleshooting

### **Camera GUI Not Found**:
If auto-detection fails:
1. Make sure `rqt_image_view` window is visible
2. Use manual region selection mode
3. Select the camera image area precisely

### **Keyboard Commands Not Working**:
1. Make sure keyboard control GUI is active/focused
2. Try toggling WASD mode (press T)
3. Test manual commands first

### **VLM Too Slow**:
1. Reduce capture FPS: `--fps 1.0`
2. Use smaller capture region
3. Check GPU utilization with `nvidia-smi`

### **No Camera Feed**:
Follow [Duckietown troubleshooting](https://docs.duckietown.com/daffy/opmanual-duckiebot/operations/make_it_see/index.html):
1. Remove camera cap
2. Check `duckiebot-interface` container is running
3. Verify ROS topics: `rostopic hz /ROBOT_NAME/camera_node/image/compressed`

## 🎯 Why This Approach is Better

1. **Leverages Existing Infrastructure**: Uses proven Duckietown tools
2. **Visual Debugging**: You can see exactly what the VLM sees
3. **Fail-Safe**: Easy to take manual control
4. **No Complex Dependencies**: Just screen capture + keyboard input
5. **Platform Independent**: Works on Windows/Linux/Mac
6. **Easy Modification**: Simple to adjust prompts and behavior

## 🚀 Advanced Usage

### Custom VLM Prompts:
Edit `get_driving_prompt()` in the script to customize behavior.

### Different Models:
Change the model in the llama.cpp Docker command to use different VLMs.

### Recording Sessions:
The system saves statistics and can be easily extended to record decisions.

### Multiple Robots:
Run multiple instances with different capture regions for multiple robots.

---

This approach is **much more practical** for testing and development than complex video streaming setups!
