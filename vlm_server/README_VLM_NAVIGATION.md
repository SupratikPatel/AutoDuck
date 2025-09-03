# DuckieBot VLM Navigation System

A comprehensive autonomous navigation system that combines Vision-Language Model (VLM) intelligence with direct robot control for DuckieBots in Duckietown environments.

## 🎯 System Overview

This system integrates the best of both worlds:
- **Video Feed**: Direct ROS camera feed from DuckieBot (like duckie_v2)
- **AI Analysis**: Qwen2.5-VL Vision-Language Model for intelligent navigation decisions
- **Robot Control**: Direct ROS control commands to robot motors (like duckie_v2)

**Replaces:**
- Screen capture → Direct camera feed
- Keyboard typing → Direct robot control
- Object detection → Lane following and navigation

## 🚀 Quick Start

### Prerequisites

1. **Docker** with GPU support
2. **ROS** environment (sourced)
3. **DuckieBot** running with camera accessible
4. **Network connection** to DuckieBot

### One-Command Setup

```bash
cd vlm_server
python start_vlm_navigation.py <ROBOT_NAME>
```

Example:
```bash
python start_vlm_navigation.py ducky
```

This will:
1. ✅ Check all prerequisites
2. 🚀 Start llama.cpp server with Qwen2.5-VL
3. 📡 Connect to DuckieBot camera feed
4. 🤖 Begin autonomous navigation

### Manual Setup

1. **Start VLM Server:**
```bash
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
  -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF --host 0.0.0.0 --port 8080 --n-gpu-layers 99
```

2. **Start Navigation:**
```bash
python duckiebot_vlm_navigation.py <ROBOT_NAME>
```

## 🎮 Operation

### Navigation Commands
The VLM analyzes camera images and generates:
- **FORWARD**: Continue straight ahead
- **LEFT**: Turn left or change to left lane
- **RIGHT**: Turn right or change to right lane  
- **STOP**: Stop for obstacles/safety

### Emergency Controls
```bash
# Emergency stop
rostopic pub /vlm_navigator/emergency_stop std_msgs/Bool 'data: true'

# Resume navigation
rostopic pub /vlm_navigator/emergency_stop std_msgs/Bool 'data: false'

# Kill system
Ctrl+C
```

### Monitoring
```bash
# Current decisions
rostopic echo /vlm_navigator/current_decision

# Performance metrics
rostopic echo /vlm_navigator/performance

# Camera feed (for debugging)
rosrun rqt_image_view rqt_image_view
```

## 🧠 AI Navigation Intelligence

### VLM Capabilities
- **Lane Following**: Stays centered between lane markings
- **Obstacle Avoidance**: Stops for obstacles, other robots, people
- **Traffic Compliance**: Stops at red stop lines
- **DuckieBot Detection**: Recognizes other robots by visual features
- **Lane Changing**: Strategic lane changes to bypass obstacles
- **Intersection Handling**: Stops, assesses, then proceeds

### Navigation Rules
1. **Safety First**: When uncertain, stops immediately
2. **Lane Discipline**: Follows yellow/white lane markings
3. **Traffic Rules**: Respects stop lines and signs
4. **Obstacle Priority**: Avoids collisions with robots/people
5. **Efficient Routing**: Prefers lane changes over stopping when safe

## 🔧 Configuration

### Command Line Options
```bash
python duckiebot_vlm_navigation.py <robot_name> [options]

Options:
  --llama-url URL     VLM server URL (default: http://localhost:8080)
  --fps FLOAT         Processing FPS (default: 2.0)
```

### Key Parameters
- **Processing FPS**: 2.0 (balance between responsiveness and load)
- **Max Speed**: 0.15 m/s (conservative for safety)
- **Max Angular**: 1.0 rad/s (smooth turning)
- **Response Timeout**: 15 seconds (safety fallback)

## 📊 Performance Monitoring

The system tracks:
- **Camera FPS**: Video feed rate from robot
- **Processing FPS**: VLM analysis rate
- **Success Rate**: Successful navigation decisions
- **Response Time**: VLM analysis latency
- **Decision Distribution**: FORWARD/LEFT/RIGHT/STOP statistics

## 🚨 Safety Features

### Automatic Safety
- **Emergency Stop**: Immediate halt on obstacles
- **Timeout Protection**: Stops if VLM fails to respond
- **Rate Limiting**: Prevents excessive processing load
- **Graceful Degradation**: Safe fallbacks for failures

### Manual Override
- **Emergency Stop Topic**: Instant remote stop capability
- **Keyboard Interrupt**: Ctrl+C for immediate shutdown
- **Auto Control Toggle**: Enable/disable autonomous mode

## 🔄 Integration with Existing Systems

### Compared to duckie_v2
- ✅ **Same**: Direct ROS camera feed and control
- ✅ **Same**: Motor control approach and safety features
- 🔄 **Different**: VLM navigation instead of object detection
- 🔄 **Different**: Lane following instead of ball following

### Compared to VLM Screen Capture
- ✅ **Same**: Qwen2.5-VL analysis and decision making
- ✅ **Same**: Dashboard and monitoring capabilities
- 🔄 **Different**: Direct camera feed instead of screen capture
- 🔄 **Different**: Robot control instead of keyboard typing

## 📁 File Structure

```
vlm_server/
├── duckiebot_vlm_navigation.py      # Main navigation system
├── start_vlm_navigation.py          # Quick startup script
├── duckiebot_vlm_navigation.launch  # ROS launch file
└── README_VLM_NAVIGATION.md         # This documentation
```

## 🐛 Troubleshooting

### Common Issues

**VLM Server Not Starting:**
```bash
# Check Docker and GPU
docker --version
nvidia-smi

# Manual server start
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda ...
```

**Camera Feed Issues:**
```bash
# Check robot connection
ping <robot_name>.local

# Check camera topics
rostopic list | grep camera

# Test camera feed
rostopic echo /<robot_name>/camera_node/image/compressed
```

**Robot Not Moving:**
```bash
# Check control topics
rostopic list | grep cmd

# Test manual control
rostopic pub /<robot_name>/car_cmd_switch_node/cmd duckietown_msgs/Twist2DStamped ...
```

### Debug Mode
```bash
# Increase logging
export ROS_LOG_LEVEL=DEBUG

# Monitor all topics
rostopic list
rostopic hz /<robot_name>/camera_node/image/compressed
```

## 🔮 Future Enhancements

- **Multi-Robot Coordination**: Handle multiple DuckieBots
- **Advanced Path Planning**: Implement A* or RRT algorithms
- **Map Integration**: Use Duckietown maps for global navigation
- **Learning Mode**: Adapt navigation based on success/failure
- **Voice Commands**: Natural language navigation instructions

## 📝 Development Notes

This system represents a significant integration achievement, combining:
1. **Real-time video processing** from robot cameras
2. **Advanced AI analysis** with vision-language models
3. **Direct robot control** via ROS messaging
4. **Comprehensive safety** and monitoring systems

The modular design allows for easy extension and adaptation to different robots, environments, and AI models.
