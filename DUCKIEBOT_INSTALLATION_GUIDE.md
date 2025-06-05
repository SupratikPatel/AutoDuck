# 🦆 DuckieBot AutoDuck VLM Installation Guide

## 📋 Overview

This guide covers complete installation of the AutoDuck VLM system on DuckieBot hardware, including both robot-side and processing server setup.

## 🎯 Prerequisites

### **Hardware Requirements**

#### **DuckieBot (Robot)**
- DuckieBot DB21M or newer
- Jetson Nano 4GB (minimum)
- Camera module (working)
- WiFi connectivity
- SD card 32GB+ (Class 10)

#### **Processing Server (Laptop/Desktop)**
- **For Ultra-Fast Performance**: RTX 4060+ GPU, 16GB+ RAM, CUDA 12.7+
- **For Stable Performance**: Any laptop with 8GB+ RAM
- WiFi connectivity (same network as robot)
- Docker Desktop (for GPU) or Python 3.8+ (for CPU)

### **Software Requirements**
- DuckieBot shell tools (`dts`)
- Git
- Docker (for robot deployment)

## 🚀 **Installation Steps**

### **Phase 1: Processing Server Setup**

Choose your implementation based on available hardware:

#### **Option A: Ultra-Fast GPU Setup**

1. **Verify GPU Support**
   ```bash
   # Check NVIDIA GPU
   nvidia-smi
   
   # Install Docker Desktop with GPU support
   # Follow: https://docs.docker.com/desktop/install/
   
   # Test GPU Docker integration
   docker run --gpus all nvidia/cuda:11.0-base nvidia-smi
   ```

2. **Start llama.cpp Server**
   ```bash
   # Pull and start GPU-accelerated server
   docker pull ghcr.io/ggml-org/llama.cpp:server-cuda
   docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
     -hf ggml-org/gemma-3-4b-it-GGUF --host 0.0.0.0 --port 8080
   
   # Wait for server to start (watch for "HTTP server listening")
   ```

3. **Clone AutoDuck Repository**
   ```bash
   git clone https://github.com/SupratikPatel/AutoDuck.git
   cd AutoDuck
   ```

4. **Start AutoDuck VLM Server**
   ```bash
   cd vlm_server/llamacpp
   pip install flask requests opencv-python pillow numpy
   python llamacpp_autoduck.py
   
   # Server starts at http://localhost:5000
   ```

#### **Option B: Stable CPU Setup**

1. **Install Ollama**
   ```bash
   # Install Ollama
   curl -fsSL https://ollama.ai/install.sh | sh
   
   # Pull Gemma3 model (this may take several minutes)
   ollama pull gemma3:4b
   
   # Verify installation
   ollama list
   ```

2. **Clone AutoDuck Repository**
   ```bash
   git clone https://github.com/SupratikPatel/AutoDuck.git
   cd AutoDuck
   ```

3. **Start AutoDuck VLM Server**
   ```bash
   cd vlm_server/ollama
   pip install flask requests ollama opencv-python pillow numpy
   python autoduck_vlm.py
   
   # Server starts at http://localhost:5000
   ```

### **Phase 2: DuckieBot Software Setup**

1. **Install DuckieBot Shell**
   ```bash
   # Install dts (if not already installed)
   pip3 install --user --upgrade duckietown-shell
   
   # Set token (get from https://hub.duckietown.com/profile)
   dts tok set YOUR_TOKEN_HERE
   ```

2. **Clone AutoDuck Repository on Development Machine**
   ```bash
   git clone https://github.com/SupratikPatel/AutoDuck.git
   cd AutoDuck
   ```

3. **Configure Network Settings**
   ```bash
   # Find your laptop IP address
   hostname -I  # Linux/Mac
   ipconfig     # Windows
   
   # Example: 192.168.1.150
   export LAPTOP_IP=192.168.1.150
   export DUCKIEBOT_NAME=duckiebot01  # Your robot name
   ```

### **Phase 3: DuckieBot Deployment**

1. **Build AutoDuck Image**
   ```bash
   # Build for ARM64 (Jetson Nano architecture)
   dts devel build -f --arch arm64v8
   
   # This builds the Docker image with VLM support
   ```

2. **Push to Registry**
   ```bash
   # Push to DuckieHub registry
   dts devel push
   ```

3. **Deploy to Robot**
   ```bash
   # Deploy and run on DuckieBot
   dts devel run -R $DUCKIEBOT_NAME
   
   # Alternative: specify laptop IP during deployment
   dts devel run -R $DUCKIEBOT_NAME -e LAPTOP_IP=$LAPTOP_IP
   ```

### **Phase 4: Launch and Configuration**

1. **Start Robot Shell**
   ```bash
   # Connect to robot
   dts start_gui_tools $DUCKIEBOT_NAME
   
   # Inside robot container
   source /ws/devel/setup.bash
   ```

2. **Launch VLM Autonomous Mode**
   ```bash
   # Method 1: Use VLM autonomous launcher
   roslaunch duckietown_demos vlm_autonomous.launch \
     veh:=$DUCKIEBOT_NAME laptop_ip:=$LAPTOP_IP
   
   # Method 2: Manual configuration
   roslaunch vlm_duckiebot_interface vlm_client.launch \
     veh:=$DUCKIEBOT_NAME \
     laptop_ip:=$LAPTOP_IP \
     fast_mode:=true \
     send_interval:=1.0
   ```

3. **Verify System Operation**
   ```bash
   # Check if robot is receiving VLM commands
   rostopic echo /$DUCKIEBOT_NAME/joy_mapper_node/car_cmd
   
   # Monitor operation mode
   rostopic echo /$DUCKIEBOT_NAME/operation_mode
   
   # Check camera feed
   rostopic hz /$DUCKIEBOT_NAME/camera_node/image/compressed
   ```

## 🎮 **Operation and Control**

### **Mode Switching**

```bash
# Switch to VLM autonomous mode
rostopic pub /$DUCKIEBOT_NAME/operation_mode std_msgs/String "data: 'vlm'" --once

# Switch to traditional lane following
rostopic pub /$DUCKIEBOT_NAME/operation_mode std_msgs/String "data: 'map'" --once

# Switch to manual control
rostopic pub /$DUCKIEBOT_NAME/operation_mode std_msgs/String "data: 'manual'" --once
```

### **Mission Control**

```bash
# Change VLM mission
rostopic pub /$DUCKIEBOT_NAME/vlm_mission std_msgs/String "data: 'explore'" --once
rostopic pub /$DUCKIEBOT_NAME/vlm_mission std_msgs/String "data: 'find_books'" --once
rostopic pub /$DUCKIEBOT_NAME/vlm_mission std_msgs/String "data: 'find_friends'" --once
```

### **Performance Monitoring**

Access the dashboard: `http://$LAPTOP_IP:5000`

- **Live camera feed**: See what robot sees
- **VLM decisions**: Real-time reasoning
- **Performance metrics**: Response times, FPS
- **System health**: Connection status

## 🛠️ **Troubleshooting**

### **Common Issues**

#### **1. Robot Not Moving**

**Check Operation Mode:**
```bash
rostopic echo /$DUCKIEBOT_NAME/operation_mode
# Should show: data: "vlm"
```

**Check Motor Commands:**
```bash
rostopic echo /$DUCKIEBOT_NAME/joy_mapper_node/car_cmd
# Should show periodic Twist2DStamped messages
```

**Solution:**
```bash
# Force switch to VLM mode
rostopic pub /$DUCKIEBOT_NAME/operation_mode std_msgs/String "data: 'vlm'" --once
```

#### **2. VLM Server Connection Issues**

**Check Server Health:**
```bash
curl http://$LAPTOP_IP:5000/api/health
# Should return: {"status": "healthy", "implementation": "llamacpp/ollama"}
```

**Check Network Connectivity:**
```bash
ping $LAPTOP_IP
# Should show successful pings
```

**Solution:**
```bash
# Restart VLM server
cd vlm_server/llamacpp  # or ollama
python llamacpp_autoduck.py  # or autoduck_vlm.py
```

#### **3. GPU Implementation Issues**

**Check GPU Status:**
```bash
nvidia-smi
# Should show GPU utilization
```

**Check Docker GPU Support:**
```bash
docker run --gpus all nvidia/cuda:11.0-base nvidia-smi
```

**Solution:**
```bash
# Restart Docker daemon
sudo systemctl restart docker

# Reinstall NVIDIA Docker support
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-docker2
```

#### **4. Camera Feed Issues**

**Check Camera Topic:**
```bash
rostopic hz /$DUCKIEBOT_NAME/camera_node/image/compressed
# Should show ~15 Hz
```

**Check Camera Node:**
```bash
rosnode list | grep camera
# Should show camera_node
```

**Solution:**
```bash
# Restart camera node
rosnode kill /$DUCKIEBOT_NAME/camera_node
# Node should automatically restart
```

### **Performance Optimization**

#### **Network Optimization**
- Use 5GHz WiFi for lower latency
- Ensure robot and laptop on same network
- Place router between robot and laptop
- Avoid network congestion during operation

#### **GPU Optimization (llama.cpp)**
- Close unnecessary applications
- Set GPU to performance mode
- Monitor temperature to prevent throttling
- Ensure adequate power supply

#### **CPU Optimization (Ollama)**
- Close memory-intensive applications
- Use SSD storage for better model loading
- Ensure adequate cooling
- Consider process priority adjustment

## 📊 **System Validation**

### **Performance Benchmarks**

**GPU Implementation (llama.cpp):**
- Target response time: < 1.5s
- Expected FPS: > 0.6
- GPU utilization: 60-80%

**CPU Implementation (Ollama):**
- Target response time: < 3.5s
- Expected FPS: > 0.25
- CPU utilization: 70-90%

### **Validation Commands**

```bash
# Test VLM server performance
curl -X POST http://$LAPTOP_IP:5000/api/analyze \
  -H "Content-Type: application/json" \
  -d '{"image_base64": "test"}'

# Monitor system performance
curl http://$LAPTOP_IP:5000/api/stats | jq

# Check robot responsiveness
rostopic hz /$DUCKIEBOT_NAME/joy_mapper_node/car_cmd
```

## 🚦 **Safety Guidelines**

1. **Always have emergency stop ready** - Keep manual control available
2. **Test in safe environment** - Use large, obstacle-free areas initially
3. **Monitor performance** - Watch dashboard for system health
4. **Network reliability** - Ensure stable WiFi connection
5. **Battery management** - Monitor robot battery levels

## 📈 **Next Steps**

1. **Test both implementations** to find optimal configuration
2. **Experiment with missions** for different behaviors
3. **Monitor performance metrics** via dashboard
4. **Scale to multiple robots** using same server
5. **Customize for specific use cases**

## 🔧 **Advanced Configuration**

### **Environment Variables**

Set these in robot deployment:

```bash
# VLM server configuration
export VLM_IMPLEMENTATION=llamacpp  # or "ollama"
export VLM_SERVER_PORT=5000
export VLM_MODEL=gemma3:4b

# Network configuration
export LAPTOP_IP=192.168.1.150
export VEHICLE_NAME=duckiebot01

# Performance tuning
export VLM_SEND_INTERVAL=1.0
export VLM_IMAGE_QUALITY=85
export VLM_RESOLUTION_WIDTH=640
export VLM_RESOLUTION_HEIGHT=480
```

### **Custom Launch Parameters**

```bash
# High-performance configuration
roslaunch vlm_duckiebot_interface vlm_client.launch \
  veh:=$DUCKIEBOT_NAME \
  laptop_ip:=$LAPTOP_IP \
  fast_mode:=true \
  send_interval:=0.8 \
  adaptive_interval:=true \
  mission:=explore \
  image_quality:=90
```

---

**🦆 Your DuckieBot is now ready for ultra-fast AI-powered autonomous navigation!**

For performance optimization and advanced features, see:
- **[README.md](README.md)** - System overview
- **[AUTODUCK_DEPLOYMENT_GUIDE.md](AUTODUCK_DEPLOYMENT_GUIDE.md)** - Deployment guide
- **[docs/VLM_DUAL_MODE_SETUP_GUIDE.md](docs/VLM_DUAL_MODE_SETUP_GUIDE.md)** - Technical setup details 