#!/bin/bash

# Setup script for Qwen 2.5 with Ollama (Recommended)

echo "Setting up Qwen 2.5 with Ollama..."

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Docker Compose is available
if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo "Docker Compose is not available. Please install Docker Compose."
    exit 1
fi

# Start the services
echo "Starting Ollama service..."
docker-compose -f docker-compose-ollama.yml up -d

# Wait for Ollama to be ready
echo "Waiting for Ollama to start..."
sleep 10

# Pull Qwen 2.5 model
echo "Pulling Qwen 2.5 model (this may take a while)..."
docker exec qwen-ollama ollama pull qwen2.5:7b

# Alternative models you can try:
# docker exec qwen-ollama ollama pull qwen2.5:1.5b
# docker exec qwen-ollama ollama pull qwen2.5:3b
# docker exec qwen-ollama ollama pull qwen2.5:14b
# docker exec qwen-ollama ollama pull qwen2.5:32b

echo "Setup complete!"
echo "Ollama API is available at: http://localhost:11434"
echo "Web UI is available at: http://localhost:3000"
echo ""
echo "Test the API with:"
echo "curl http://localhost:11434/api/generate -d '{\"model\":\"qwen2.5:7b\",\"prompt\":\"Hello, how are you?\"}'"
echo ""
echo "To stop the services, run: docker-compose -f docker-compose-ollama.yml down"
