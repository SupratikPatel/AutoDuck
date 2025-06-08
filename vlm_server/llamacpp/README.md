# AutoDuck llama.cpp Integration with Anti-Deadlock Navigation

**🚗 Ultra-Fast VLM Processing with Smart Obstacle Avoidance**

## 🎯 **Overview**

This directory contains the llama.cpp integration for AutoDuck, featuring:
- **Qwen2.5-VL-7B** vision language model via Docker
- **Anti-deadlock navigation system** (prevents infinite STOP loops)
- **Decision memory** (prevents oscillation patterns)
- **Real-time performance monitoring** with comprehensive session analysis
- **Web dashboard** for monitoring navigation decisions

## 🚀 **Quick Start**

### **1. Start llama.cpp Server**
```bash
# Optimized for RTX 4060 8GB VRAM
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 \
    --n-gpu-layers 99 --ctx-size 1024 --batch-size 256 --threads 4 --cont-batching
```

### **2. Start AutoDuck VLM**
```bash
cd vlm_server/llamacpp
python3 llamacpp_autoduck.py
```

### **3. Access Dashboard**
```
Browser: http://localhost:5000
```

## 🧠 **Anti-Deadlock Navigation System**

### **Problem Solved**
Traditional VLM systems get stuck in infinite STOP loops when encountering obstacles:
- **Before**: 70%+ STOP decisions (robot gets stuck)
- **After**: <20% STOP decisions (smooth navigation)

### **How It Works**

#### **1. Smart Obstacle Avoidance Prompts**
```python
"DECISION PRIORITY RULES:
1. FORWARD: Only if the path ahead is completely clear and safe
2. LEFT: If there's an obstacle ahead but the left side looks clearer/safer
3. RIGHT: If there's an obstacle ahead but the right side looks clearer/safer
4. STOP: ONLY for immediate extreme danger

OBSTACLE AVOIDANCE STRATEGY:
- If you see a static obstacle: Choose LEFT or RIGHT to go around it
- Always try to find a way around obstacles
- Prefer LEFT/RIGHT over STOP for navigation"
```

#### **2. Anti-Deadlock Logic**
```python
def apply_anti_deadlock_logic(self, decision):
    # Count consecutive STOPs
    if decision == 'STOP':
        self.consecutive_stops += 1
    
    # If too many consecutive STOPs, force alternative navigation
    if self.consecutive_stops >= 3:
        print("🚨 Anti-deadlock: 3 consecutive STOPs detected!")
        print("   → Forcing RIGHT/LEFT to bypass obstacle")
        return 'RIGHT' if self.consecutive_stops % 2 == 1 else 'LEFT'
```

#### **3. Anti-Oscillation Detection**
```python
# Check for LEFT-RIGHT oscillation
if len(self.last_decisions) >= 4:
    recent_4 = self.last_decisions[-4:]
    left_count = recent_4.count('LEFT')
    right_count = recent_4.count('RIGHT')
    
    # If oscillating between LEFT and RIGHT, try FORWARD
    if left_count >= 2 and right_count >= 2:
        print("🔄 Anti-oscillation: Detected LEFT-RIGHT oscillation, trying FORWARD")
        return 'FORWARD'
```

## 📊 **Performance Monitoring**

### **Real-Time Stats**
The system tracks comprehensive navigation metrics:

```python
# Performance checkpoints every 10 requests
📊 Performance Checkpoint (Request #50):
   ✅ Success Rate: 50/50 (100.0%)
   ⚡ Avg Response: 0.85s
   📈 Current FPS: 1.18
   🎮 Recent Decisions: ['FORWARD', 'LEFT', 'FORWARD']
```

### **Final Session Summary**
When you quit with 'Q', you get a comprehensive analysis:

```
============================================================
🏁 FINAL AUTODUCK VLM SESSION SUMMARY
============================================================

📊 PERFORMANCE OVERVIEW:
   🎯 Total Requests: 182
   ✅ Successful: 182
   ❌ Failed: 0
   📈 Success Rate: 100.0%

⚡ RESPONSE TIME ANALYSIS:
   🎯 Average: 0.85s
   🏆 Best: 0.75s
   📉 Worst: 1.54s
   🚀 Best FPS: 1.34

🎮 NAVIGATION DECISION ANALYSIS:
   FORWARD: 116 (63.7%)
   LEFT: 16 (8.8%)
   RIGHT: 16 (8.8%)
   STOP: 34 (18.7%)

🧭 NAVIGATION QUALITY ASSESSMENT:
   🎯 Low STOP rate (18.7%) - Excellent navigation flow
   🚗 Movement: 63.7% forward, 17.6% maneuvering
   🔄 Final 5 decisions: STOP → STOP → FORWARD → FORWARD → LEFT

🏆 OVERALL PERFORMANCE RATING:
   🥈 GOOD (1.2 FPS) - Near real-time performance
============================================================
```

## 🎮 **Controls**

| Key | Action | Description |
|-----|--------|-------------|
| **SPACE** | Single Analysis | Analyze current frame once |
| **C** | Continuous Mode | Toggle continuous analysis on/off |
| **Q** | Quit | Exit with comprehensive summary |

## 🔧 **Configuration**

### **Optimized Settings for RTX 4060**
```python
# Performance optimizations
self.image_quality = 85      # Balanced quality vs speed
self.max_tokens = 50         # Sufficient for navigation decisions
self.temperature = 0.4       # Balanced creativity/consistency

# Anti-deadlock parameters
consecutive_stops_limit = 3   # Force navigation after 3 STOPs
decision_memory_size = 5     # Track last 5 decisions
oscillation_detection = 4    # Check last 4 for LEFT-RIGHT loops
```

### **Docker Command Variations**

**Standard Performance** (RTX 4060):
```bash
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 \
    --n-gpu-layers 99 --ctx-size 1024 --batch-size 256 --threads 4 --cont-batching
```

**Alternative Model** (smaller, faster):
```bash
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-3B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 \
    --n-gpu-layers 99 --ctx-size 1024 --batch-size 256 --threads 4 --cont-batching
```

## 🛠️ **Troubleshooting**

### **High STOP Rate (>30%)**
```bash
# Check if anti-deadlock is working
# Look for these messages in output:
# 🚨 Anti-deadlock: 3 consecutive STOPs detected!
#    → Forcing RIGHT to bypass obstacle

# If not appearing, check:
# 1. Obstacle placement (should be clearly visible)
# 2. Lighting conditions
# 3. Camera focus and resolution
```

### **Low FPS Performance (<1 FPS)**
```bash
# Check GPU utilization
nvidia-smi

# Try reducing context size
docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 --port 8080 \
    --n-gpu-layers 99 --ctx-size 512 --batch-size 128 --threads 4
```

### **Server Connection Issues**
```bash
# Test server health
curl http://localhost:8080/health

# Check if port is in use
netstat -tlnp | grep 8080

# Restart Docker container
docker ps  # Find container ID
docker stop <container_id>
docker run ...  # Restart with command above
```

## 📈 **Performance Benchmarks**

| Metric | Target | Achieved | Status |
|--------|--------|----------|---------|
| **FPS** | >1.0 | 1.2+ | ✅ |
| **STOP Rate** | <30% | 18.7% | ✅ |
| **Success Rate** | >95% | 100% | ✅ |
| **Forward Movement** | >50% | 63.7% | ✅ |
| **Response Time** | <1.5s | 0.85s | ✅ |

## 🔮 **Features**

- **✅ Anti-Deadlock System**: Prevents infinite STOP loops
- **✅ Decision Memory**: Tracks patterns to avoid oscillation
- **✅ Real-Time Dashboard**: Web interface at localhost:5000
- **✅ Comprehensive Analysis**: Detailed session summaries
- **✅ Performance Monitoring**: FPS, response time, decision quality
- **✅ Smart Navigation**: 63.7% forward movement achieved
- **✅ Robust Error Handling**: Graceful failure recovery

## 🚀 **Next Steps**

1. **Deploy to DuckieBot**: Integrate with robot navigation stack
2. **Fine-tune Parameters**: Adjust for specific environments
3. **Add More Models**: Test other vision language models
4. **Enhance Dashboard**: Add real-time charts and metrics
5. **Logging System**: Add persistent performance logging

---

**🦆 Ready to navigate smarter with AutoDuck!** 