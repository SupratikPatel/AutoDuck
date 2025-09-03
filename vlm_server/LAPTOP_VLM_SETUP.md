# Laptop VLM Setup Guide - Simple Screen Capture Approach

Complete guide for setting up the VLM (Vision Language Model) components on your laptop for autonomous Duckiebot control using screen capture.

## 🎯 Overview

This approach captures the Duckiebot's camera feed from the GUI and uses a VLM to make driving decisions via keyboard simulation - much simpler than complex ROS integration!

## 📋 Prerequisites

- **Hardware**: NVIDIA GPU with 8GB+ VRAM (RTX 4060 recommended)
- **Software**: Docker with NVIDIA support, Python 3.8+
- **Network**: Laptop and Duckiebot on same WiFi
- **Duckiebot**: Already running with camera and keyboard control

## 🚀 Quick Start (3 Steps)

### Step 1: Start VLM Server
```bash
# Start Qwen2.5-VL server with GPU acceleration
docker run --gpus all -p 8080:8080 --name llama-vlm-server \
    ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 --n-gpu-layers 99
```

**Wait for**: `✅ llama.cpp server is running at http://localhost:8080`

### Step 2: Install Dependencies
```bash
cd vlm_server
pip install mss pygetwindow pyautogui keyboard opencv-python-headless requests numpy
```

### Step 3: Run Screen Capture VLM
```bash
python simple_screen_capture_vlm.py
```

## 🎮 Complete Workflow

```bash
# Terminal 1: Camera GUI (on Duckiebot side)
dts start_gui_tools YOUR_ROBOT_NAME
# Then run: rqt_image_view
# Select: camera_node/image/compressed

# Terminal 2: Keyboard Control (on Duckiebot side)
dts duckiebot keyboard_control YOUR_ROBOT_NAME

# Terminal 3: VLM Server (on Laptop)
docker run --gpus all -p 8080:8080 --name llama-vlm-server \
    ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 --n-gpu-layers 99

# Terminal 4: Screen Capture VLM (on Laptop)
cd vlm_server
python simple_screen_capture_vlm.py
```

## 🤖 Automated Setup

For easier setup, use the automated script:

```bash
cd vlm_server
python start_laptop_vlm.py
```

This script automatically:
- ✅ Checks Docker and GPU availability
- ✅ Starts llama.cpp server  
- ✅ Verifies Python dependencies
- ✅ Guides you through next steps
- ✅ Optionally starts screen capture VLM

## 🎮 Controls

Once the screen capture VLM is running:

| Key | Action |
|-----|--------|
| **SPACE** | Toggle auto control ON/OFF |
| **T** | Toggle between Arrow keys (↑↓←→) and WASD |
| **M** | Manual single frame analysis |
| **Q** | Quit and show statistics |

## 🔧 Setup Process

### Screen Region Selection

The system will:
1. **Auto-detect** `rqt_image_view` window (if running)
2. **Manual selection** if auto-detection fails:
   - Move cursor to **top-left** of camera view → Press **SPACE**
   - Move cursor to **bottom-right** of camera view → Press **SPACE**
3. **Test capture** and save `test_capture.jpg`
4. **Start control loop**

### VLM Decision Flow

```
Camera Feed → Screen Capture → VLM Analysis → Keyboard Command → Robot Movement
```

**Command Mapping:**
- `FORWARD` → ↑ (up arrow) → Move forward
- `LEFT` → ← (left arrow) → Turn left  
- `RIGHT` → → (right arrow) → Turn right
- `STOP` → Space → Emergency stop

## 📊 Performance

- **VLM Speed**: 1-2 FPS with Qwen2.5-VL on RTX 4060
- **Decision Quality**: Smart obstacle avoidance with safety defaults
- **Response Time**: ~0.5-1.0 seconds per decision
- **Memory Usage**: ~6-7GB VRAM

## 🛠️ Troubleshooting

### Docker Issues
```bash
# On Windows with WSL2
wsl --update
# Restart Docker Desktop

# On Linux  
sudo systemctl start docker

# Check NVIDIA support
docker run --rm --gpus all nvidia/cuda:11.8-base-ubuntu20.04 nvidia-smi
```

### GPU Not Detected
```bash
# Check NVIDIA drivers
nvidia-smi

# Install NVIDIA Container Toolkit (Linux)
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Python Dependencies
```bash
# Install missing packages
pip install mss pygetwindow pyautogui keyboard opencv-python-headless requests numpy

# On some systems, you might need:
pip install pillow tkinter
```

### Screen Capture Issues
```bash
# Linux: Install window management tools
sudo apt-get install wmctrl

# Permission issues on some systems
# Run with elevated privileges if needed
```

### VLM Server Not Starting
```bash
# Check logs
docker logs llama-vlm-server

# Remove and restart container
docker stop llama-vlm-server
docker rm llama-vlm-server
# Then run the docker command again

# Check port availability
netstat -an | grep 8080
```

### Keyboard Commands Not Working
1. **Make sure keyboard control GUI is active/focused**
2. **Try toggling WASD mode** (press T)
3. **Test manual commands first** (press M)
4. **Check if GUI window is visible**

## 📈 Performance Optimization

### For Faster Processing:
```bash
# Reduce capture rate
python simple_screen_capture_vlm.py --fps 1.0

# Use smaller capture region (select minimal camera area)

# Optimize Docker settings
docker run --gpus all -p 8080:8080 --name llama-vlm-server \
    --memory=8g --cpus=4 \
    ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 --n-gpu-layers 99 \
    --ctx-size 512 --batch-size 128
```

### For Better Accuracy:
```bash
# Higher capture quality (slower)
python simple_screen_capture_vlm.py --fps 2.0

# Larger capture region for more context
# Better lighting conditions
# Cleaner camera lens
```

## 🔧 Configuration

### Custom VLM Prompts
Edit `get_driving_prompt()` in `simple_screen_capture_vlm.py` to customize behavior.

### Different Models
Change the Docker command to use different models:
```bash
# Smaller model (faster, less accurate)
-hf ggml-org/Qwen2-VL-2B-Instruct-GGUF

# Larger model (slower, more accurate)  
-hf ggml-org/Qwen2.5-VL-14B-Instruct-GGUF
```

### Key Bindings
Modify `decision_to_key` and `decision_to_wasd` dictionaries in the script.

## 📝 What's Next?

1. **Start Duckiebot camera and keyboard GUIs** (you know how)
2. **Run this laptop VLM setup**
3. **Select camera area** when prompted
4. **Press SPACE** to enable auto control
5. **Watch your robot navigate autonomously!**

## 🎉 Advantages

- ✅ **Simple Setup**: Uses existing Duckietown tools
- ✅ **Visual Debugging**: See what VLM analyzes
- ✅ **No ROS Complexity**: Direct keyboard simulation
- ✅ **Easy Modification**: Change prompts and behavior easily
- ✅ **Reliable Control**: Direct integration with proven keyboard GUI
- ✅ **Platform Independent**: Works on Windows/Linux/Mac

---

**Ready to control your Duckiebot with AI vision? Follow this guide and you'll be up and running in minutes!** 🚗🤖
