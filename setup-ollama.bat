@echo off
REM Setup script for Qwen 2.5 with Ollama (Windows)

echo Setting up Qwen 2.5 with Ollama...

REM Check if Docker is running
docker version >nul 2>&1
if %errorlevel% neq 0 (
    echo Docker is not running. Please start Docker Desktop first.
    pause
    exit /b 1
)

REM Start the services
echo Starting Ollama service...
docker-compose -f docker-compose-ollama.yml up -d

REM Wait for Ollama to be ready
echo Waiting for Ollama to start...
timeout /t 10 /nobreak

REM Pull Qwen 2.5 model
echo Pulling Qwen 2.5 model (this may take a while)...
docker exec qwen-ollama ollama pull qwen2.5:7b

echo Setup complete!
echo Ollama API is available at: http://localhost:11434
echo Web UI is available at: http://localhost:3000
echo.
echo Test the API with:
echo curl http://localhost:11434/api/generate -d "{\"model\":\"qwen2.5:7b\",\"prompt\":\"Hello, how are you?\"}"
echo.
echo To stop the services, run: docker-compose -f docker-compose-ollama.yml down
pause
