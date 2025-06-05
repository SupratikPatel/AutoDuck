# 🤖 AutoDuck VLM - Ollama Stable Implementation

**Stable autonomous driving system with proven reliability**

## 📊 Performance Results

- **Response Time**: 3.20s average (stable)
- **Best Response**: 3.00s
- **FPS**: 0.27
- **Success Rate**: 100%
- **Decision Quality**: Good (with FORWARD bias elimination)

## 🔧 System Requirements

- **Ollama**: Latest version installed locally
- **Model**: Gemma3:4b-vision
- **Memory**: 8GB+ RAM recommended
- **Python**: 3.8+ with required packages

## 🚀 Quick Start

### 1. Install Ollama and Model

```bash
# Install Ollama (if not already installed)
curl -fsSL https://ollama.ai/install.sh | sh

# Pull the vision model
ollama pull gemma3:4b
```

### 2. Run AutoDuck VLM System

```bash
# Main system with integrated dashboard
python autoduck_vlm.py

# Access at: http://localhost:5000
```

## 📁 Files

- **`autoduck_vlm.py`** - Main stable VLM system with integrated dashboard
- **`fast_realtime_webcam.py`** - Alternative faster implementation (reference)

## ⚡ Key Features

- **Proven Stability**: Long-term tested and reliable
- **Local Processing**: No external dependencies
- **Integrated Dashboard**: Real-time monitoring
- **Safety First**: Conservative decision making
- **Easy Setup**: Simple Ollama-based deployment

## 🎯 Use Cases

- **Development**: Great for testing and development
- **Backup System**: Reliable fallback option
- **Local Deployment**: When internet connectivity is limited
- **Learning**: Understanding VLM systems

## 🔗 Dependencies

- ollama (local installation)
- flask
- cv2 (opencv-python)
- numpy
- PIL
- requests

## 📈 Technical Details

- **Model**: Gemma3:4b vision model
- **Resolution**: 640x480 (balanced)
- **JPEG Quality**: 85%
- **Max Tokens**: 20
- **Temperature**: 0.4
- **API**: Ollama REST API

## 🔄 Migration to llama.cpp

For 3x better performance, consider migrating to the `../llamacpp/` implementation:
- Sub-second response times
- CUDA GPU acceleration
- Better throughput for robot deployment 