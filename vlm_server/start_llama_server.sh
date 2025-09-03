#!/bin/bash
# Start llama.cpp server with Qwen2.5-VL for Duckiebot control

echo "🚀 Starting llama.cpp server with Qwen2.5-VL-7B..."
echo "📊 Optimized for RTX 4060 (8GB VRAM)"

# Check if container already exists
if docker ps -a | grep -q llama-vlm-server; then
    echo "⚠️  Removing existing container..."
    docker stop llama-vlm-server 2>/dev/null
    docker rm llama-vlm-server 2>/dev/null
fi

# Start the server
docker run -d \
    --gpus all \
    -p 8080:8080 \
    --name llama-vlm-server \
    --restart unless-stopped \
    ghcr.io/ggml-org/llama.cpp:server-cuda \
    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \
    --host 0.0.0.0 \
    --port 8080 \
    --n-gpu-layers 99 \
    --ctx-size 1024 \
    --batch-size 256 \
    --threads 4 \
    --cont-batching

echo "⏳ Waiting for server to start..."
sleep 5

# Check if server is running
for i in {1..30}; do
    if curl -s http://localhost:8080/health > /dev/null 2>&1; then
        echo "✅ llama.cpp server is running at http://localhost:8080"
        echo "📊 Model: Qwen2.5-VL-7B with full GPU acceleration"
        echo ""
        echo "📋 Next steps:"
        echo "1. Start keyboard control bridge: python keyboard_control_bridge.py"
        echo "2. Launch complete system: python launch_vlm_keyboard_control.py YOUR_ROBOT_NAME"
        exit 0
    fi
    echo -n "."
    sleep 2
done

echo ""
echo "❌ Failed to start llama.cpp server"
echo "Check logs with: docker logs llama-vlm-server"
exit 1
