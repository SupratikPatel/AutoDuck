# VLM Keyboard Control System for Duckiebot

This system enables autonomous Duckiebot control using Vision-Language Models (VLM) through keyboard input simulation. It processes the sssssrobot's camera feed on your powerful laptop GPU and sends keyboard commands to the existing Duckietown keyboard control interface.

## 🚀 Architecture Overview

```what
┌─────────────────────────────────────────────────────────────────┐
│                        LAPTOP (RTX 4060)                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌───────────────┐    ┌──────────────────┐    ┌─────────────┐ │
│  │  llama.cpp    │    │ Keyboard Control │    │   Video     │ │
│  │   Server      │◄───│     Bridge       │◄───│  Streamer   │ │
│  │ (Qwen2.5-VL)  │    │  (FastAPI)       │    │   (ROS)     │ │
│  └───────────────┘    └──────────────────┘    └─────────────┘ │
│         │                      │                       ▲        │
│         │                      │                       │        │
│         └──────────────────────┘                      │        │
│                      │                                 │        │
│                      ▼                                 │        │
│              ┌──────────────┐                          │        │
│              │  Keyboard    │                          │        │
│              │    GUI       │                          │        │
│              └──────────────┘                          │        │
│                      │                                 │        │
└──────────────────────┼─────────────────────────────────┼────────┘
                       │                                 │
                       │ Keyboard                        │ Camera
                       │ Commands                        │ Feed
                       ▼                                 │
┌─────────────────────────────────────────────────────────────────┐
│                         DUCKIEBOT                               │
│  ┌─────────────┐    ┌──────────────┐    ┌──────────────┐      │
│  │   Motors    │◄───│ Joy Mapper   │    │   Camera     │      │
│  └─────────────┘    └──────────────┘    └──────────────┘      │
└─────────────────────────────────────────────────────────────────┘
```

## 🛠️ Components

### 1. **VLM Keyboard Control Bridge** (`keyboard_control_bridge.py`)
- FastAPI server that receives video frames
- Processes them through llama.cpp VLM
- Converts decisions (FORWARD/LEFT/RIGHT/STOP) to keyboard inputs
- Simulates keyboard presses on the Duckietown GUI

### 2. **Duckiebot VLM Streamer** (`duckiebot_vlm_streamer.py`)
- ROS node that connects to Duckiebot's camera
- Streams video to the control bridge via WebSocket/HTTP
- Monitors performance and FPS

### 3. **Integrated Launcher** (`launch_vlm_keyboard_control.py`)
- Automatically starts all components
- Manages llama.cpp Docker container
- Handles cleanup on shutdown

## 📋 Requirements

### On Your Laptop:
- Ubuntu 20.04+ (or Windows with WSL2)
- NVIDIA GPU with 8GB+ VRAM (RTX 4060 or better)
- Docker with NVIDIA Container Toolkit
- Python 3.8+
- ROS Noetic (for camera streaming)
- Duckietown Shell (`dts`)

### Python Dependencies:
```bash
pip install fastapi uvicorn websocket-client opencv-python-headless \
            pyautogui keyboard requests numpy pillow
```

### On Windows (additional):
```bash
pip install pywin32
```

## 🚀 Quick Start

### 1. **Start the Complete System**
```bash
cd vlm_server
python launch_vlm_keyboard_control.py YOUR_DUCKIEBOT_NAME
```

This will:
- Start llama.cpp server with Qwen2.5-VL
- Launch the keyboard control bridge
- Start video streaming from Duckiebot
- Open the keyboard control GUI

### 2. **Access the Dashboard**
Open your browser: http://localhost:8000

### 3. **Enable VLM Control**
1. Make sure the keyboard GUI window is visible
2. Click "Activate Keyboard GUI" in the dashboard
3. Toggle "Auto Control: ON"

## 🎮 Manual Component Launch

If you prefer to start components individually:

### 1. **Start llama.cpp Server**
```bash
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 \
    --n-gpu-layers 99 --ctx-size 1024 --batch-size 256
```

### 2. **Start Keyboard Control Bridge**
```bash
cd vlm_server
python keyboard_control_bridge.py
```

### 3. **Start Video Streamer**
```bash
export ROS_MASTER_URI=http://YOUR_DUCKIEBOT_NAME.local:11311
export ROS_IP=YOUR_LAPTOP_IP
python duckiebot_vlm_streamer.py YOUR_DUCKIEBOT_NAME
```

### 4. **Launch Keyboard GUI**
```bash
dts duckiebot keyboard_control YOUR_DUCKIEBOT_NAME
```

## 🔧 Configuration

### Keyboard Control Bridge Settings
Edit `keyboard_control_bridge.py`:
```python
# Key press duration (seconds)
self.key_press_duration = 0.1  

# Time between decisions (seconds)
self.decision_interval = 0.5

# Key mapping
self.decision_to_key = {
    'FORWARD': 'w',     # or 'up'
    'LEFT': 'a',        # or 'left'
    'RIGHT': 'd',       # or 'right'
    'STOP': 's',        # or 'space'
}
```

### VLM Prompts
Customize the driving prompt in `get_driving_prompt()`:
```python
def get_driving_prompt(self) -> str:
    return """Your custom prompt here..."""
```

## 📊 Performance Optimization

### For Better FPS:
1. **Reduce image resolution**:
   ```python
   # In duckiebot_vlm_streamer.py
   cv_image = cv2.resize(cv_image, (320, 240))
   ```

2. **Adjust decision interval**:
   ```python
   # In keyboard_control_bridge.py
   self.decision_interval = 1.0  # Slower but more stable
   ```

3. **Use faster VLM model**:
   Consider smaller GGUF quantizations of Qwen2.5-VL

### For Better Accuracy:
1. **Increase image quality**:
   ```python
   cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
   ```

2. **Enhance prompts** with more specific instructions

3. **Add decision smoothing** to prevent oscillations

## 🐛 Troubleshooting

### "Keyboard GUI not responding"
1. Make sure the GUI window is active/focused
2. On Linux: Install `wmctrl` for window activation
3. Try manual control first to verify GUI works

### "No video feed"
1. Check ROS_MASTER_URI points to Duckiebot
2. Verify camera topic: `rostopic list | grep camera`
3. Test with: `rostopic hz /YOUR_ROBOT/camera_node/image/compressed`

### "VLM decisions too slow"
1. Check GPU usage: `nvidia-smi`
2. Reduce image size or quality
3. Increase decision interval

### "WebSocket connection failed"
1. Use HTTP fallback: `--no-websocket` flag
2. Check firewall settings
3. Verify bridge is running on port 8000

## 🎯 Usage Tips

1. **Start Simple**: Test with manual controls first
2. **Monitor Performance**: Watch FPS and decision latency
3. **Adjust Prompts**: Fine-tune for your specific track
4. **Safety First**: Always be ready to take manual control

## 🔍 Monitoring

### Dashboard Features:
- Real-time camera view
- Current VLM decision display
- Performance metrics (FPS, latency)
- Decision history
- Manual control buttons

### Command Line Monitoring:
```bash
# Watch ROS topics
rostopic echo /YOUR_ROBOT/joy_mapper_node/car_cmd

# Monitor system performance
htop
nvidia-smi -l 1
```

## 🚨 Safety Notes

1. **Always supervise** the robot during autonomous operation
2. **Start with low speeds** by adjusting motor parameters
3. **Have emergency stop** ready (Space key or E-stop)
4. **Test in safe environment** before complex scenarios

## 📝 Advanced Features

### Custom Decision Logic
Add to `parse_vlm_decision()`:
```python
# Add custom rules
if "red light" in content.lower():
    return 'STOP'
elif "intersection" in content.lower():
    # Custom intersection logic
    pass
```

### Recording Sessions
The system tracks all decisions in `stats['decision_history']` for later analysis.

### Integration with Other Systems
The FastAPI server provides endpoints for integration with other tools:
- `/process_frame` - Process single frame
- `/control/manual` - Send manual commands
- `/stats` - Get performance statistics

## 🤝 Contributing

Feel free to enhance the system:
- Add more sophisticated decision logic
- Implement path planning integration
- Add support for multiple robots
- Improve GUI window detection

## 📄 License

This project extends the Duckietown framework and follows its licensing terms.
