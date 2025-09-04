# 🤖 AutoDuck - Vision Language Model for Duckiebot Navigation

**Advanced AI-powered autonomous navigation system using Vision Language Models (VLM) for Duckiebot robots.**

## 🎯 **Overview**

AutoDuck is an autonomous navigation system that uses Vision Language Models (VLM) to enable Duckiebot robots to navigate complex environments with human-like understanding. The system can detect and avoid obstacles, follow lanes, and make intelligent navigation decisions in real-time.

### ✨ **Key Features**

- 🧠 **Advanced VLM Integration**: Uses Qwen2.5-VL-7B for superior vision understanding
- 🦆 **Duckiebot-Specific Detection**: Recognizes other duckiebots, yellow rubber ducks, and red stop lines
- 📊 **Real-time Dashboard**: Live monitoring and control interface
- ⚡ **High Performance**: >1.2 FPS with optimized GPU acceleration
- 🎮 **Dual Mode Operation**: Webcam and screen capture modes
- 🔧 **Easy Setup**: One-command deployment with Docker

## 🚀 **Quick Start**

### **Prerequisites**

- **NVIDIA GPU** (RTX 4060 or better recommended)
- **Docker** with NVIDIA Container Toolkit
- **Python 3.8+**
- **Linux/macOS**

### **1. Clone the Repository**

```bash
git clone https://github.com/SupratikPatel/AutoDuck.git
cd AutoDuck

```

### **2. Create and Activate Virtual Environment**

```bash
# Create virtual environment
python -m venv .venv

# Activate virtual environment
# On Windows:
.venv\Scripts\activate
# Or with PowerShell:
.venv\Scripts\Activate.ps1

# On Linux/macOS:
source .venv/bin/activate

# To deactivate when done:
deactivate
```

### **3. Install Python Dependencies**

```bash
pip install -r requirements.txt
```

### **4. Start the VLM Server**

**For RTX 4060 (Optimized Settings):**
```bash
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 \
    --n-gpu-layers 99 --ctx-size 1024 --batch-size 256 --threads 4 --cont-batching
```

**For Other GPUs:**
```bash
# RTX 3060 (Lower VRAM)
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-3B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 \
    --n-gpu-layers 99 --ctx-size 512 --batch-size 128 --threads 2 --cont-batching
```

### **5. Launch AutoDuck System**

```bash
cd vlm_server
python start_both_vlm_systems.py
```

### **Alternative Setup: Client/Server inside DuckieBot container**

If you prefer running the DuckieBot-side code from inside the `duckietown/dt-duckiebot-interface` container and a separate processing server, see `clientServer/README.md`.

High-level steps:
- SSH into the DuckieBot, find and `docker exec` into the interface container
- Clone this repo inside the container and run `./run_autonomous_driving.sh`
- On your PC/laptop, run `python server/autonomous_server_qwen_docker.py` (after updating IPs)

### **6. Select Your Mode**

Choose from the menu:
- **Option 1**: Webcam VLM (localhost:5000) - Direct camera input
- **Option 2**: Screen Capture VLM (localhost:3000) - Analyze any video/screen
- **Option 3**: Both systems simultaneously

## 📁 **Project Structure & Workflow**

```
duckieT/
├── vlm_server/                          # Main VLM System (Screen Capture)
│   ├── llamacpp/
│   │   └── llamacpp_autoduck.py        # Webcam VLM with Flask dashboard
│   ├── screen_capture_vlm_with_dashboard.py  # Screen capture VLM
│   └── start_both_vlm_systems.py       # Launcher script
├── clientServer/                        # Alternative: Client/Server Architecture
│   ├── autonomous_driving_dashboard.py  # Web monitoring dashboard
│   ├── run_autonomous_driving.sh       # DuckieBot launcher
│   ├── src/
│   │   ├── object_follower/
│   │   │   └── scripts/
│   │   │       └── autonomous_detector.py  # ROS node for DuckieBot
│   │   └── duckietown_msgs/            # ROS message definitions
│   └── README.md                       # Client/Server setup guide
└── README.md                           # This file - Main documentation
```

### **🔄 Workflow Overview**

#### **Method 1: Screen Capture VLM**
```
1. Start VLM Server (Docker) → localhost:8080
2. Launch Screen Capture VLM → localhost:3000
3. Select DuckieBot camera area on screen
4. VLM analyzes captured area → Generates navigation commands
5. Commands sent to DuckieBot keyboard control GUI
6. DuckieBot executes movements (WASD/Arrow keys)
```

#### **Method 2: Client/Server Architecture**
```
1. Start VLM Server (Docker) → localhost:8080
2. Start Processing Server → localhost:8000
3. Launch DuckieBot ROS node → Connects to processing server
4. DuckieBot camera → Processing server → VLM analysis
5. Commands sent back to DuckieBot via ROS
6. DuckieBot executes autonomous movements
```

### **🎯 Key Components**

| Component | Purpose | Port | Method |
|-----------|---------|------|--------|
| **VLM Server** | Qwen2.5-VL inference | 8080 | Docker |
| **Screen Capture VLM** | Screen analysis + keyboard control | 3000 | Python |
| **Webcam VLM** | Direct camera analysis | 5000 | Python |
| **Processing Server** | Client/Server bridge | 8000 | Python |
| **DuckieBot ROS** | Robot control interface | - | ROS |

## 🎥 **Screen Capture Mode Setup**

When using **Screen Capture VLM** (Option 2):

1. **Open your video source:**
   - YouTube video of Duckiebot navigation
   - Live Duckiebot camera stream
   - Any video file with road/obstacle scenarios

2. **Select capture area:**
   - The system will guide you to select the video region
   - Use SPACE to mark corners of the video area
   - Press ESC to cancel selection

3. **Start analysis:**
   - The VLM will analyze the selected area in real-time
   - View decisions and performance metrics on the dashboard

## 🧠 **VLM Model Options**

### **Recommended Models**

| Model | Size | VRAM | Performance | Use Case |
|-------|------|------|-------------|----------|
| **Qwen2.5-VL-7B** | 7B | 8GB+ | ⭐⭐⭐⭐⭐ | **Recommended** - Best balance |
| apple/FastVLM-1.5B | 1.5B | 4GB+ | ⭐⭐⭐⭐ | Lower-end GPUs |
| Gemma-3-27b | 27B | 17GB+ | ⭐⭐⭐⭐⭐ | High-end GPUs |

### **Alternative Models**

**For different use cases, you can use other GGUF models:**

```bash
# Llama 3.2 Vision (Alternative)
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf microsoft/Llama-3.2-11B-Vision-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 --n-gpu-layers 99

# LLaVA (Lightweight)
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf microsoft/llava-v1.6-mistral-7b-GGUF \
    --host 0.0.0.0 --port 8080 --n-gpu-layers 99
```

**Find more models at:** [Hugging Face GGUF Models](https://huggingface.co/models?library=gguf&sort=trending)

## 🐳 **Recommended Deployment: Dockerized LLMs**

### **Why Dockerized LLMs?**

**✅ Advantages:**
- **Easy Setup**: One-command deployment
- **GPU Optimization**: Automatic CUDA/MLX integration
- **Isolation**: No dependency conflicts
- **Portability**: Works across different systems
- **Performance**: Optimized inference engines

### **Platform-Specific Recommendations**

#### **🪟 Windows (NVIDIA GPU)**
**Preferred: Docker with CUDA support**
```bash
# Install Docker Desktop with NVIDIA Container Toolkit
# Then run optimized GGUF models:
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 \
    --n-gpu-layers 99 --ctx-size 1024 --batch-size 256 --threads 4 --cont-batching
```

**Benefits:**
- Full GPU acceleration with CUDA
- Optimized memory management
- Easy model switching
- Production-ready performance

#### **🍎 macOS (Apple Silicon)**
**Preferred: MLX-optimized Docker**
```bash
# For Apple Silicon Macs (M1/M2/M3)
docker run -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-mlx \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 \
    --mlx --ctx-size 1024 --batch-size 256 --threads 8
```

**Benefits:**
- Native Apple Silicon optimization
- Unified memory architecture
- Excellent performance on M-series chips
- Lower power consumption

#### **🐧 Linux (NVIDIA/AMD)**
**Preferred: Docker with platform-specific GPU support**
```bash
# NVIDIA GPUs
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 --n-gpu-layers 99

# AMD GPUs (ROCm)
docker run --device=/dev/kfd --device=/dev/dri --group-add video -p 8080:8080 \
    ghcr.io/ggml-org/llama.cpp:server-rocm \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 --n-gpu-layers 99
```

### **🚫 Avoid These Approaches**

**❌ Local Python Installation:**
- Complex dependency management
- Platform-specific compilation issues
- Harder to maintain and update
- Potential CUDA version conflicts

**❌ Ollama (for this project):**
- Slower
- Less control over inference parameters
- Not optimized for real-time robotics

**✅ Stick with Dockerized GGUF/MLX models for:**
- Maximum performance
- Easy deployment
- Cross-platform compatibility
- Production reliability

### **Customizing Model Parameters**

**GPU Memory Optimization:**
```bash
# Adjust based on your GPU VRAM
--n-gpu-layers 99    # Use all GPU layers (8GB+ VRAM)
--n-gpu-layers 50    # Use half GPU layers (4GB VRAM)
--n-gpu-layers 25    # Use quarter GPU layers (2GB VRAM)
```

**Context Size (Memory vs Performance):**
```bash
--ctx-size 2048      # High performance, more VRAM
--ctx-size 1024      # Balanced (RTX 4060 optimal)
--ctx-size 512       # Lower VRAM usage
```

**Batch Size (Throughput vs Latency):**
```bash
--batch-size 512     # High throughput, more VRAM
--batch-size 256     # Balanced (RTX 4060 optimal)
--batch-size 128     # Lower latency, less VRAM
```

## 🎮 **Usage Guide**

### **Webcam Mode (localhost:5000)**
- Direct camera input for real-time navigation
- Perfect for live Duckiebot control
- Real-time performance monitoring

### **Screen Capture Mode (localhost:3000)**
- Analyze any video source (YouTube, live streams, files)
- Great for testing and development
- Select specific screen regions for analysis

### **Controls- Cool Dashboard in localhost:3000**
- **SPACE**: Toggle continuous analysis
- **C**: Toggle continuous mode
- **Q**: Quit application
- **M**: Manual single frame capture

## 🦆 **Detection Capabilities**

### **Primary Detection Targets**
- **Duckiebots**: Blue chassis, yellow wheels
- **Yellow Rubber Ducks**: Classic duck toys and duck-shaped objects
- **Red Lines**: Stop lines and red tape markings
- **Other Obstacles**: People, vehicles, barriers

### **Detection Range**
- **10cm proximity detection** for all targets
- **Smart lane-changing** when obstacles detected
- **Intelligent navigation** with obstacle avoidance

## 🛣️ **Navigation Strategy**

### **Obstacle Avoidance Protocol**
1. **STOP** when obstacle detected within 10cm
2. **LEFT** to position in left lane
3. **RIGHT** to navigate around obstacle
4. **RIGHT** to return to original lane
5. **FORWARD** to continue navigation


## 🔧 **Troubleshooting**

### **Common Issues**

**VLM Server Not Starting:**
```bash
# Check Docker and GPU
docker --version
nvidia-smi

# Restart with verbose logging
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 --n-gpu-layers 99 --verbose
```

**Low Performance:**
```bash
# Monitor GPU usage
nvidia-smi -l 1

# Adjust parameters for your GPU
# Reduce ctx-size and batch-size for lower VRAM GPUs
```

**Connection Issues:**
```bash
# Test server connectivity
curl http://localhost:8080/health

# Check firewall settings
# Ensure ports 8080, 5000, 3000 are open
```

### **Performance Optimization**

**For RTX 4060 (8GB VRAM):**
```bash
--n-gpu-layers 99 --ctx-size 1024 --batch-size 256 --threads 4
```

**For RTX 3060 (6GB VRAM):**
```bash
--n-gpu-layers 99 --ctx-size 512 --batch-size 128 --threads 2
```

## 📊 **Dashboard Features**

### **Real-time Monitoring**
- Live video feed with overlay
- Performance metrics (FPS, response time)
- Decision history and statistics
- Navigation quality assessment

### **Control Interface**
- Manual/Automatic mode toggle
- Continuous capture control
- Performance optimization settings
- Error logging and diagnostics

## 🎯 **Advanced Configuration**

### **Customizing Detection Parameters**

Edit the system prompts in:
- `vlm_server/llamacpp/llamacpp_autoduck.py`
- `vlm_server/screen_capture_vlm_with_dashboard.py`

**Key Parameters:**
- Detection distance (currently 10cm)
- Obstacle types and priorities
- Navigation strategies
- Performance thresholds

### **Adding New Detection Targets**

1. Update the system prompt with new object descriptions
2. Add detection examples and reasoning
3. Test with various scenarios
4. Optimize detection parameters

## 🤝 **Contributing**

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 **License**

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 **Acknowledgments**

- **Qwen Team** for the excellent Vision Language Model
- **llama.cpp** for the high-performance inference engine
- **Duckietown** for the autonomous vehicle platform
- **OpenCV** for computer vision capabilities


