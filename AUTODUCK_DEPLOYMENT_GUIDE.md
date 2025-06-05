# 🦆 AutoDuck VLM Deployment Guide

## 📋 Overview

This guide covers deploying the AutoDuck VLM (Vision Language Model) system on DuckieBots with full ROS integration. The system supports both ultra-fast GPU acceleration (llama.cpp) and stable CPU inference (Ollama).

## 🏗️ Architecture

```
DuckieBot (Robot)                    Processing Server (Laptop/Desktop)
┌─────────────────────┐             ┌─────────────────────────────────┐
│ ROS Nodes           │             │ VLM Server                      │
│ ├── Camera Node     │────Image────│ ├── llama.cpp (GPU) 1.2s       │
│ ├── VLM Client      │◄──Decision──│ └── Ollama (CPU) 3.0s          │
│ ├── Motor Control   │             │                                 │
│ └── Joy Mapper      │             │ Web Dashboard (localhost:5000)  │
└─────────────────────┘             └─────────────────────────────────┘
```

## 🚀 Quick Deployment

### 1. Build & Deploy Robot Container

```bash
# Build the DuckieBot image with VLM support
dts devel build -f --arch arm64v8

# Push to registry
dts devel push

# Deploy to robot
dts devel run -R ROBOT_NAME
```

### 2. Start VLM Processing Server

Choose your implementation:

**For Ultra-Fast Performance (GPU Required):**
```bash
# On processing laptop/desktop with RTX GPU
cd vlm_server/llamacpp
python llamacpp_autoduck.py
```

**For Stable CPU Performance:**
```bash
# On any laptop/desktop
cd vlm_server/ollama  
python autoduck_vlm.py
```

### 3. Launch Robot in VLM Mode

```bash
# SSH to robot
dts start_gui_tools ROBOT_NAME

# Launch VLM autonomous mode
roslaunch duckietown_demos vlm_autonomous.launch veh:=ROBOT_NAME laptop_ip:=LAPTOP_IP
```

## 🔧 Configuration Options

### Environment Variables

Set these in your robot container or docker-compose:

```bash
# VLM Implementation Selection
VLM_IMPLEMENTATION=llamacpp  # or "ollama"
VLM_SERVER_PORT=5000
VLM_MODEL=gemma3:4b

# Network Configuration  
LAPTOP_IP=192.168.1.150
VEHICLE_NAME=duckiebot01
```

### Launch Parameters

```bash
# Basic VLM autonomous mode
roslaunch duckietown_demos vlm_autonomous.launch \
    veh:=duckiebot01 \
    laptop_ip:=192.168.1.150

# High-performance mode with custom settings
roslaunch vlm_duckiebot_interface vlm_client.launch \
    veh:=duckiebot01 \
    laptop_ip:=192.168.1.150 \
    fast_mode:=true \
    send_interval:=1.0 \
    mission:=explore
```

## 📦 Dependencies & Integration

### Added to `dependencies-py3.txt`:
```
flask>=2.0.0
requests>=2.25.0  
opencv-python>=4.5.0
pillow>=8.0.0
ollama>=0.1.0
python-dateutil
```

### Updated `Dockerfile`:
- Added VLM system dependencies
- Environment variable support
- VLM server code copying
- Python package installations

### New Launcher Scripts:
- `launchers/vlm_autonomous.sh` - Starts VLM system
- `packages/duckietown_demos/launch/vlm_autonomous.launch` - ROS integration

## 🎮 Operation Modes

### Switching Modes

```bash
# Switch to VLM autonomous mode
rostopic pub /duckiebot01/operation_mode std_msgs/String "data: 'vlm'" --once

# Switch back to manual control
rostopic pub /duckiebot01/operation_mode std_msgs/String "data: 'manual'" --once

# Change mission dynamically
rostopic pub /duckiebot01/vlm_mission std_msgs/String "data: 'find_books'" --once
```

### Available Missions
- `explore` - General safe exploration (default)
- `find_books` - Look for books and text objects
- `find_friends` - Search for other robots
- `navigate` - Efficient navigation
- `clean` - Look for cluttered areas

## 📊 Performance Comparison

| Implementation | Response Time | FPS | Hardware Required | Best For |
|---------------|---------------|-----|-------------------|----------|
| **llama.cpp** | **1.19s** | **0.82** | RTX GPU + CUDA | **Real-time control** |
| **Ollama** | 3.20s | 0.27 | Any CPU | Stable deployment |

## 🛠️ Troubleshooting

### Common Issues

1. **VLM Server Not Found**
   ```bash
   # Check server status
   curl http://LAPTOP_IP:5000/api/health
   
   # Verify network connectivity
   ping LAPTOP_IP
   ```

2. **GPU Not Detected (llama.cpp)**
   ```bash
   # Check CUDA installation
   nvidia-smi
   
   # Verify Docker GPU support
   docker run --gpus all nvidia/cuda:11.0-base nvidia-smi
   ```

3. **Robot Not Moving**
   ```bash
   # Check operation mode
   rostopic echo /duckiebot01/operation_mode
   
   # Monitor motor commands
   rostopic echo /duckiebot01/joy_mapper_node/car_cmd
   ```

### Debug Commands

```bash
# Monitor VLM decisions
rostopic echo /duckiebot01/vlm_debug

# Check camera feed
rostopic hz /duckiebot01/camera_node/image/compressed

# View performance stats
curl http://LAPTOP_IP:5000/api/stats | jq
```

## 🌐 Web Dashboard

Access the real-time dashboard at: `http://LAPTOP_IP:5000`

Features:
- Live camera feed
- VLM decisions and reasoning
- Performance metrics
- Processing time graphs
- Decision distribution stats

## 🔒 Security Considerations

1. **Network Security**: Ensure VLM server is on trusted network
2. **Firewall**: Open port 5000 on processing machine
3. **Authentication**: Consider adding API keys for production

## 📚 Integration Points

### ROS Topics
- **Input**: `/ROBOT/camera_node/image/compressed`
- **Output**: `/ROBOT/joy_mapper_node/car_cmd`
- **Control**: `/ROBOT/operation_mode`
- **Mission**: `/ROBOT/vlm_mission`

### Launch Files
- `vlm_autonomous.launch` - Full autonomous mode
- `vlm_client.launch` - VLM client only
- `lane_following.launch` - Traditional mode (disabled in VLM)

### Docker Integration
- Multi-architecture support (arm64v8 for robot)
- Environment variable configuration
- Automatic service discovery
- Graceful shutdown handling

## 🚦 Safety Features

1. **Automatic Stopping**: On server timeout or error
2. **Mode Switching**: Instant switch between autonomous/manual
3. **Obstacle Detection**: AprilTag and object detection remain active
4. **Network Monitoring**: Automatic reconnection attempts
5. **Performance Monitoring**: Adaptive frame rate based on processing time

## 📈 Next Steps

1. **Multi-Robot Support**: Scale to robot swarms
2. **Edge Deployment**: On-robot GPU acceleration
3. **Mission Planning**: Advanced task coordination
4. **Fleet Management**: Centralized control dashboard

---

## 🎯 Quick Start Commands

```bash
# 1. Deploy to robot
dts devel build -f --arch arm64v8 && dts devel push && dts devel run -R ROBOT_NAME

# 2. Start VLM server (choose one)
python vlm_server/llamacpp/llamacpp_autoduck.py     # GPU acceleration
python vlm_server/ollama/autoduck_vlm.py           # CPU stable

# 3. Launch autonomous mode
roslaunch duckietown_demos vlm_autonomous.launch veh:=ROBOT_NAME laptop_ip:=LAPTOP_IP

# 4. Access dashboard
open http://LAPTOP_IP:5000
```

**🦆 Your DuckieBot is now ready for AI-powered autonomous exploration!** 