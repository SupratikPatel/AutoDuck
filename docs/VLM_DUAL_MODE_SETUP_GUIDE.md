# AutoDuck VLM Ultra-Fast Setup Guide

## 🚀 Overview

AutoDuck now features **dual VLM implementations** for maximum performance and compatibility:
- **🚀 llama.cpp GPU**: Ultra-fast processing (1.19s response times)
- **🤖 Ollama CPU**: Stable baseline (3.20s response times)

Both implementations use the same **Gemma-3-4b** vision model with full ROS integration.

## 📊 **Performance Comparison**

| Implementation | Response Time | FPS | Hardware Required | Best For |
|---------------|---------------|-----|-------------------|----------|
| **🚀 llama.cpp** | **1.19s** | **0.82** | RTX 4060+ GPU + CUDA | **Real-time control** |
| **🤖 Ollama** | 3.20s | 0.27 | Any CPU | Stable deployment |

## 🚀 **Quick Start**

### **Option 1: Ultra-Fast GPU Setup (Recommended)**

#### **Prerequisites**
- NVIDIA RTX 4060+ GPU
- Docker Desktop with GPU support
- CUDA 12.7+ drivers

#### **Setup**
```bash
# 1. Start llama.cpp Docker server
docker pull ghcr.io/ggml-org/llama.cpp:server-cuda
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
  -hf ggml-org/gemma-3-4b-it-GGUF --host 0.0.0.0 --port 8080

# 2. Start AutoDuck VLM server
cd vlm_server/llamacpp
python llamacpp_autoduck.py

# 3. Access dashboard
open http://localhost:5000
```

### **Option 2: Stable CPU Setup (Any Hardware)**

#### **Prerequisites**
- Any laptop/desktop with 8GB+ RAM
- Python 3.8+

#### **Setup**
```bash
# 1. Install Ollama
curl -fsSL https://ollama.ai/install.sh | sh

# 2. Pull Gemma3 model
ollama pull gemma3:4b

# 3. Start AutoDuck VLM server
cd vlm_server/ollama
python autoduck_vlm.py

# 4. Access dashboard
open http://localhost:5000
```

## 🤖 **DuckieBot Integration**

### **1. Build and Deploy**
```bash
# Build DuckieBot image with VLM support
dts devel build -f --arch arm64v8

# Deploy to robot
dts devel push
dts devel run -R DUCKIEBOT_NAME
```

### **2. Launch VLM Mode**
```bash
# SSH to robot
dts start_gui_tools DUCKIEBOT_NAME

# Launch VLM autonomous navigation
roslaunch duckietown_demos vlm_autonomous.launch \
  veh:=DUCKIEBOT_NAME laptop_ip:=LAPTOP_IP
```

### **3. Control Robot**
```bash
# Switch to VLM autonomous mode
rostopic pub /DUCKIEBOT_NAME/operation_mode std_msgs/String "data: 'vlm'" --once

# Change mission
rostopic pub /DUCKIEBOT_NAME/vlm_mission std_msgs/String "data: 'explore'" --once

# Switch back to traditional mode
rostopic pub /DUCKIEBOT_NAME/operation_mode std_msgs/String "data: 'map'" --once
```

## 🎮 **Available Missions**

- **`explore`**: General safe exploration (default)
- **`find_books`**: Look for books and text objects
- **`find_friends`**: Search for other robots or people
- **`navigate`**: Efficient point-to-point navigation
- **`clean`**: Look for cluttered areas to organize

## 🔧 **System Architecture**

### **llama.cpp Implementation**
```
Camera → Base64 → HTTP → Docker llama.cpp → OpenAI API → Flask → Dashboard
                         (GPU Accelerated)
```

### **Ollama Implementation**
```
Camera → Base64 → HTTP → Local Ollama → Python API → Flask → Dashboard
                         (CPU Inference)
```

### **ROS Integration**
```
DuckieBot Camera → VLM Client → Auto-Detect Server → Send Commands → Robot Motors
```

## 📊 **Performance Monitoring**

### **Real-time Dashboard** (`http://localhost:5000`)
- **Live camera feed**: See robot's view
- **VLM decisions**: Real-time reasoning
- **Performance metrics**: Response times, FPS
- **Decision stats**: FORWARD/LEFT/RIGHT/STOP distribution
- **System health**: Server status and errors

### **API Endpoints**
```bash
# Check system health
curl http://localhost:5000/api/health

# Get performance statistics
curl http://localhost:5000/api/stats

# View latest decision
curl http://localhost:5000/api/latest
```

### **Command Line Monitoring**
```bash
# Monitor llama.cpp implementation
cd vlm_server/llamacpp
python llamacpp_autoduck.py --verbose

# Monitor Ollama implementation
cd vlm_server/ollama
python autoduck_vlm.py --verbose
```

## 🛠️ **Configuration**

### **Environment Variables**
```bash
# Choose implementation
export VLM_IMPLEMENTATION=llamacpp  # or "ollama"
export VLM_SERVER_PORT=5000
export VLM_MODEL=gemma3:4b

# Network configuration
export LAPTOP_IP=192.168.1.150
export VEHICLE_NAME=duckiebot01
```

### **Launch File Parameters**
```bash
# High-performance mode
roslaunch vlm_duckiebot_interface vlm_client.launch \
  veh:=duckiebot01 \
  laptop_ip:=192.168.1.150 \
  fast_mode:=true \
  send_interval:=1.0 \
  mission:=explore
```

## 🔍 **Troubleshooting**

### **GPU Implementation Issues**

**1. Docker GPU not detected**
```bash
# Check GPU support
nvidia-smi
docker run --gpus all nvidia/cuda:11.0-base nvidia-smi

# Verify Docker GPU integration
docker run --gpus all hello-world
```

**2. llama.cpp server not starting**
```bash
# Check Docker logs
docker logs $(docker ps -q --filter ancestor=ghcr.io/ggml-org/llama.cpp:server-cuda)

# Manual container debugging
docker run --gpus all -it --entrypoint /bin/bash ghcr.io/ggml-org/llama.cpp:server-cuda
```

### **CPU Implementation Issues**

**1. Ollama model not found**
```bash
# Check available models
ollama list

# Re-pull model
ollama pull gemma3:4b

# Test model directly
ollama run gemma3:4b "Hello world"
```

**2. Slow performance**
```bash
# Check system resources
htop

# Monitor Ollama logs
ollama logs
```

### **Robot Integration Issues**

**1. Robot not responding**
```bash
# Check operation mode
rostopic echo /duckiebot01/operation_mode

# Monitor motor commands
rostopic echo /duckiebot01/joy_mapper_node/car_cmd

# Test manual control
rostopic pub /duckiebot01/operation_mode std_msgs/String "data: 'manual'" --once
```

**2. Network connectivity**
```bash
# Test VLM server connection
curl http://LAPTOP_IP:5000/api/health

# Check robot connectivity
ping duckiebot01.local

# Verify camera feed
rostopic hz /duckiebot01/camera_node/image/compressed
```

## 🚦 **Safety Considerations**

1. **Emergency Stop**: Always have manual control ready
2. **Network Monitoring**: System stops on connection loss
3. **Performance Monitoring**: Adaptive frame rates prevent overload
4. **Obstacle Detection**: AprilTag and YOLO detection remain active
5. **Speed Limits**: Conservative speeds for safety

## 📈 **Performance Optimization**

### **For llama.cpp (GPU)**
- Ensure CUDA drivers are up to date
- Use high-performance GPU power mode
- Close unnecessary applications to free VRAM
- Monitor GPU temperature and throttling

### **For Ollama (CPU)**
- Close memory-intensive applications
- Use fast SSD storage for model files
- Ensure adequate cooling for sustained performance
- Consider upgrading to higher-core CPU

### **Network Optimization**
- Use 5GHz WiFi for lower latency
- Ensure strong signal strength to robot
- Place router between laptop and robot
- Consider dedicated network for robot communication

## 🎯 **Next Steps**

1. **Test both implementations** to find what works best for your hardware
2. **Monitor performance** using the dashboard
3. **Experiment with missions** to explore different behaviors
4. **Scale to multiple robots** using the same server
5. **Customize prompts** for specific use cases

---

**🦆 Ready for ultra-fast autonomous robot control!**

For deployment instructions, see **[AUTODUCK_DEPLOYMENT_GUIDE.md](../AUTODUCK_DEPLOYMENT_GUIDE.md)**
