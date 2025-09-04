# DuckieBot Autonomous Driving with Qwen Docker

A streamlined autonomous driving system for DuckieBot using Qwen 2.5 VL vision model running in Docker. Based on the [AutoDuck project](https://github.com/SupratikPatel/AutoDuck.git) with enhanced safety features and duck detection.

## 🚗 Features

- **Lane Following**: Detects yellow/white lane lines and follows Duckietown tracks
- **Stop Line Detection**: Recognizes red stop lines and stops appropriately  
- **Duck Safety**: Emergency stop when ducks/ducklings are detected
- **Vision Intelligence**: Uses Qwen 2.5 VL for sophisticated scene understanding
- **Real-time Control**: High-performance ROS architecture with 5-second continuous movement
- **Web Dashboard**: Real-time monitoring and control interface
- **Anti-Deadlock Navigation**: Prevents getting stuck in loops (STOP rate <20%)

## 🏗️ Architecture

```
DuckieBot (ROS) ←→ Processing Server ←→ Qwen Docker (Windows/vLLM)
```

## 🚀 Complete Setup Guide

### Prerequisites

- **Windows PC** with NVIDIA GPU (for Qwen Docker)
- **Linux Server/Laptop** (for processing server)
- **DuckieBot** with ROS setup
- **Docker Desktop** on Windows
- **Python 3.8+** on all systems

### Step 1: Clone Repository

**On your Windows PC (Laptop):**
```bash
git clone https://github.com/SupratikPatel/AutoDuck.git
cd AutoDuck/server
```

**On your Linux Server (if different from Windows):**
```bash
git clone https://github.com/SupratikPatel/AutoDuck.git
cd AutoDuck/server
```

### Step 2: Setup Qwen Docker (Windows PC)

1. **Install Docker Desktop** with NVIDIA GPU support
2. **Run Qwen Docker Container:**
```cmd
# Navigate to AutoDuck directory
cd AutoDuck

# Start Qwen 2.5 VL Docker container
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 \
    --n-gpu-layers 99 --ctx-size 1024 --batch-size 256 --threads 4 --cont-batching
```

3. **Verify Qwen is running:**
   - Open browser: `http://localhost:8080/health`
   - Should return: `{"status": "ok"}`

### Step 3: Setup Processing Server

**On your Linux Server/Laptop:**
```bash
cd AutoDuck/server

# Install Python dependencies
pip install -r autonomous_requirements.txt

# Find your laptop's IP address
ipconfig  # Windows
# or
ip addr show  # Linux

# Update IP addresses in configuration files:
# 1. Edit server/autonomous_server_qwen_docker.py (line 32)
# 2. Replace "192.168.140.179" with your laptop's IP

# Start the processing server
python autonomous_server_qwen_docker.py
```

**Expected output:**
```
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### Step 4: Setup DuckieBot

**SSH into your DuckieBot:**
```bash
ssh duckie@[DUCKIEBOT_NAME].local
```

**Clone repository on DuckieBot:**
```bash
git clone https://github.com/SupratikPatel/AutoDuck.git
cd AutoDuck
```

**Update IP configuration:**
```bash
# Edit src/object_follower/scripts/autonomous_detector.py (line 22)
# Replace "192.168.140.179" with your laptop's IP address
nano src/object_follower/scripts/autonomous_detector.py
```

**Install ROS dependencies:**
```bash
# Make scripts executable
chmod +x run_autonomous_driving.sh

# Install Python dependencies (if needed)
pip install -r server/duckiebot_requirements.txt
```

### Step 5: Launch the System

**1. Start Qwen Docker (Windows PC):**
```cmd
# Keep the Docker container running
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 \
    --n-gpu-layers 99 --ctx-size 1024 --batch-size 256 --threads 4 --cont-batching
```

**2. Start Processing Server (Linux):**
```bash
cd AutoDuck/server
python autonomous_server_qwen_docker.py
```

**3. Launch DuckieBot (on DuckieBot):**
```bash
cd AutoDuck
./run_autonomous_driving.sh
```

**4. Monitor with Dashboard (Optional):**
```bash
# On your laptop
cd AutoDuck
python autonomous_driving_dashboard.py
# Dashboard: http://localhost:4000
```

### Alternative: Run from inside DuckieBot interface container

If your DuckieBot uses the `duckietown/dt-duckiebot-interface` container, you can run the autonomous driving stack directly from within that container:

**1) Connect to the DuckieBot**
```bash
ssh duckie@[DUCKIEBOT_NAME].local
```
Replace `[DUCKIEBOT_NAME]` with your DuckieBot's hostname.

**2) Find and enter the interface container**
```bash
docker ps
```
Locate the container with image `duckietown/dt-duckiebot-interface` and note its Container ID, then:
```bash
docker exec -it [CONTAINER_ID] /bin/bash
```

**3) Clone this repository inside the container**
```bash
git clone https://github.com/SupratikPatel/AutoDuck.git
cd AutoDuck
```

**4) Launch the DuckieBot autonomous driving**
```bash
./run_autonomous_driving.sh
```

In parallel, on your PC/laptop, run the processing server so the DuckieBot can call it:
```bash
python server/autonomous_server_qwen_docker.py
```

Make sure you have already updated the IPs in `server/autonomous_server_qwen_docker.py` and `src/object_follower/scripts/autonomous_detector.py` to point to your PC/laptop as described above.

## 📁 Project Structure

```
AutoDuck/
├── server/
│   ├── autonomous_server_qwen_docker.py    # Main processing server
│   ├── autonomous_requirements.txt         # Server Python dependencies
│   ├── duckiebot_requirements.txt          # DuckieBot Python dependencies
│   └── test_qwen_docker_api.py            # API testing
├── src/
│   ├── object_follower/
│   │   ├── scripts/autonomous_detector.py  # ROS node
│   │   └── launch/autonomous_driving.launch
│   └── duckietown_msgs/                   # ROS message definitions
├── autonomous_driving_dashboard.py        # Web dashboard
├── run_autonomous_driving.sh              # DuckieBot launcher
├── start_qwen_docker_server.sh            # Server launcher
├── run_qwen_docker.bat                    # Windows Docker launcher
└── DOCKER_SETUP.md                        # Detailed setup guide
```

## 🔧 Configuration

**Critical: Update IP addresses before running!**

1. **Find your laptop's IP address:**
   ```bash
   ipconfig  # Windows
   ip addr show  # Linux
   ```

2. **Update these files:**
   - `server/autonomous_server_qwen_docker.py` (line 32)
   - `src/object_follower/scripts/autonomous_detector.py` (line 22)

3. **Replace `192.168.140.179` with your actual IP address**

## 🛡️ Safety Features

- **Emergency Stop**: Activated when ducks/ducklings detected
- **Persistent Stop**: Robot stays stopped until duck leaves camera view
- **5cm Safety Zone**: Immediate stop for objects within 5cm
- **Continuous Movement**: 5-second movement cycles for smooth operation
- **Anti-Deadlock**: Prevents getting stuck in navigation loops
- **Network Safety**: Automatic stop on connection loss

## 📊 Performance

- **Processing Speed**: ~5 seconds per analysis cycle
- **Movement**: Continuous 5-second movement periods
- **Safety**: Real-time duck detection and emergency stops
- **Monitoring**: Live dashboard with performance metrics
- **STOP Rate**: <20% (excellent navigation flow)
- **FPS**: >1.2 FPS with Qwen2.5-VL-7B

## 🎯 Key Files

- **Main Server**: `server/autonomous_server_qwen_docker.py`
- **ROS Node**: `src/object_follower/scripts/autonomous_detector.py`
- **Dashboard**: `autonomous_driving_dashboard.py`
- **Setup Guide**: `DOCKER_SETUP.md`

## 🚨 Troubleshooting

### Common Issues

**1. Qwen Docker Not Starting:**
```bash
# Check GPU availability
nvidia-smi

# Test with smaller model
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-3B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080
```

**2. Connection Issues:**
```bash
# Test server connectivity
curl http://YOUR_LAPTOP_IP:8000/health

# Check DuckieBot can reach server
ssh duckie@[DUCKIEBOT_NAME].local
curl http://YOUR_LAPTOP_IP:8000/health
```

**3. High STOP Rate:**
- Check obstacle placement and lighting
- Ensure clear lane markings
- Monitor dashboard for navigation patterns

**4. Performance Issues:**
- Reduce Docker context size: `--ctx-size 512`
- Use smaller model: `Qwen2.5-VL-3B-Instruct-GGUF`
- Close other GPU applications

## 🦆 DuckieTown Safety

The system prioritizes DuckieTown inhabitant safety:
- Detects yellow rubber ducks, ducklings, and duckiebots
- Implements immediate emergency stops
- Maintains safety until ducks are out of camera view
- Logs all safety events for monitoring
- Anti-deadlock navigation prevents dangerous loops

## 🎉 Success Indicators

When everything is working correctly, you should see:

**Qwen Docker:**
```
Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8080
```

**Processing Server:**
```
INFO:     Uvicorn running on http://0.0.0.0:8000
✅ Qwen Docker connection successful
```

**DuckieBot:**
```
🚀 Starting autonomous driving...
📡 Connected to server at http://YOUR_IP:8000
🤖 DuckieBot is now autonomous!
```

**Dashboard:**
```
🌐 Dashboard running at http://localhost:4000
📊 Real-time monitoring active
```

Your DuckieBot is now a responsible citizen of DuckieTown! 🦆🤖🛡️

## 📚 Additional Resources

- **Detailed Setup**: See `DOCKER_SETUP.md` for comprehensive instructions
- **API Testing**: Use `server/test_qwen_docker_api.py` to test connections
- **Performance Monitoring**: Dashboard provides real-time metrics
- **Source Project**: Based on [AutoDuck](https://github.com/SupratikPatel/AutoDuck.git)
