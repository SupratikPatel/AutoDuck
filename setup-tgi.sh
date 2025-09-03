#!/bin/bash

# Setup script for Qwen 2.5 with HuggingFace TGI

echo "Setting up Qwen 2.5 with HuggingFace Text Generation Inference..."

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if NVIDIA Docker is available
if ! docker run --rm --gpus all nvidia/cuda:11.7-base-ubuntu20.04 nvidia-smi &> /dev/null; then
    echo "NVIDIA Docker support is not available. Please install NVIDIA Container Toolkit."
    echo "Visit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html"
    exit 1
fi

# Start the TGI service
echo "Starting TGI service (this will download the model automatically)..."
docker-compose -f docker-compose-tgi.yml up -d

echo "Waiting for TGI to download model and start..."
echo "This may take several minutes depending on your internet connection..."

# Wait for the service to be ready
while ! curl -s http://localhost:8080/health > /dev/null; do
    echo "Waiting for TGI to be ready..."
    sleep 10
done

echo "Setup complete!"
echo "TGI API is available at: http://localhost:8080"
echo ""
echo "Test the API with:"
echo "curl http://localhost:8080/generate \\"
echo "  -X POST \\"
echo "  -d '{\"inputs\":\"Hello, how are you?\",\"parameters\":{\"max_new_tokens\":100}}' \\"
echo "  -H 'Content-Type: application/json'"
echo ""
echo "To stop the service, run: docker-compose -f docker-compose-tgi.yml down"
