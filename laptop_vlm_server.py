#!/usr/bin/env python3

import base64
import io
import logging
import datetime # For timestamping

import ollama  # Official Ollama Python client
import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from PIL import Image  # For verifying image data if needed, optional
from pydantic import BaseModel  # For data validation
from typing import Optional

# --- Configuration ---
# Updated to support both LLaVA and Gemma 3 models
AVAILABLE_MODELS = {
    "gemma3-4b": "gemma3:4b-instruct",           # Recommended for RTX 4060
    "gemma3-12b": "gemma3:12b-instruct",         # Higher accuracy, needs more VRAM
    "llava-7b": "llava:7b-v1.6",                # Original LLaVA model
    "llava-13b": "llava:13b-v1.6"               # Larger LLaVA model
}

# Default model (Gemma 3 4B for best performance)
DEFAULT_MODEL = "gemma3-4b"
OLLAMA_VLM_MODEL_NAME = AVAILABLE_MODELS[DEFAULT_MODEL]
CURRENT_MODEL_TYPE = DEFAULT_MODEL

OLLAMA_API_HOST = None 

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- Pydantic Models ---
class VLMProcessingRequest(BaseModel):
    image_base64: str
    prompt: str = "Analyze this robot's camera view. What is the safest and most logical next action for a small robot to explore the area? Choose one: FORWARD, LEFT, RIGHT, or STOP."
    fast_mode: bool = False  # Enable fast mode for simple commands only
    model: str = DEFAULT_MODEL  # NEW: Allow model selection

class RobotCommandResponse(BaseModel):
    action: str
    speed: float = 0.5
    reasoning: str = ""
    raw_vlm_output: str = ""
    processing_time: float = 0.0
    model_used: str = DEFAULT_MODEL  # NEW: Track which model was used

class ModelSwitchRequest(BaseModel):
    model: str

# --- FastAPI Application and Templating ---
app = FastAPI(title="Duckiebot VLM Processing Server with Gemma 3", version="0.4.0")

# Path to the templates directory (assuming it's in the same directory as this script)
templates = Jinja2Templates(directory="templates")

# --- In-memory store for the latest data for GUI ---
latest_data_for_gui = {
    "timestamp": None,
    "received_image_base64": None,
    "prompt_sent": None,
    "vlm_raw_output": None,
    "command_sent": None, 
    "error": None,
    "processing_time": None,
    "fast_mode": False,
    "current_model": DEFAULT_MODEL
}

# --- Ollama Client Initialization ---
try:
    ollama_client = ollama.Client(host=OLLAMA_API_HOST)
    logger.info(f"Attempting to connect to Ollama and check for model: '{OLLAMA_VLM_MODEL_NAME}'...")
    ollama_client.show(OLLAMA_VLM_MODEL_NAME)
    logger.info(f"Successfully connected to Ollama. Model '{OLLAMA_VLM_MODEL_NAME}' is available.")
    logger.info(f"Current model type: {CURRENT_MODEL_TYPE}")
except ollama.ResponseError as e:
    logger.error(f"Ollama model '{OLLAMA_VLM_MODEL_NAME}' not found or Ollama service error: {e}")
    logger.error(f"Please ensure Ollama is running and you have pulled the model (e.g., `ollama pull {OLLAMA_VLM_MODEL_NAME}`).")
    logger.info("Available models to try:")
    for key, model in AVAILABLE_MODELS.items():
        logger.info(f"  {key}: ollama pull {model}")
    ollama_client = None
except Exception as e:
    logger.error(f"Failed to initialize or connect to Ollama client: {e}")
    ollama_client = None

def get_optimized_prompt(fast_mode: bool = False, mission: str = "explore", model_type: str = DEFAULT_MODEL) -> str:
    """Generate optimized prompts based on mode, mission, and model type"""
    
    # Gemma 3 optimized prompts (better at following instructions)
    if model_type.startswith("gemma3"):
        if fast_mode:
            return (
                f"<image>\n"
                f"You are controlling a small autonomous robot. Mission: {mission}.\n"
                f"Analyze this camera image and respond with ONLY ONE WORD: FORWARD, LEFT, RIGHT, or STOP.\n"
                f"Choose the safest path to avoid obstacles and continue the mission."
            )
        else:
            return (
                f"<image>\n"
                f"You are an autonomous robot with mission: {mission}.\n"
                f"Analyze this camera view carefully. Consider obstacles, paths, objects of interest, and safety.\n"
                f"Decide your next action (FORWARD, LEFT, RIGHT, or STOP) and explain your reasoning.\n"
                f"Be curious but prioritize safety above all."
            )
    
    # LLaVA prompts (original format)
    else:
        if fast_mode:
            return (
                f"You are controlling a small robot. Look at this image and decide the next move. "
                f"Mission: {mission}. "
                f"Respond with only ONE WORD: FORWARD, LEFT, RIGHT, or STOP. "
                f"Choose the safest option to avoid obstacles."
            )
        else:
            return (
                f"You are an autonomous robot explorer with the mission to {mission}. "
                f"Analyze this camera view and decide your next action. "
                f"Consider: obstacles, clear paths, interesting objects, and safety. "
                f"Respond with your decision (FORWARD, LEFT, RIGHT, or STOP) and briefly explain your reasoning. "
                f"Be curious but prioritize safety."
            )

def parse_vlm_output_to_command(vlm_text_output: str, fast_mode: bool = False, model_type: str = DEFAULT_MODEL) -> tuple[str, str]:
    """Enhanced parsing for both fast and detailed modes across different models"""
    processed_text = vlm_text_output.upper()
    action = "stop"  # Default to safe action
    
    # Primary action detection (works for both Gemma 3 and LLaVA)
    if "FORWARD" in processed_text or "STRAIGHT" in processed_text or "AHEAD" in processed_text:
        action = "forward"
    elif "LEFT" in processed_text:
        action = "left" 
    elif "RIGHT" in processed_text:
        action = "right"
    elif "STOP" in processed_text or "HALT" in processed_text or "WAIT" in processed_text:
        action = "stop"
    else:
        logger.warning(f"Could not parse clear command from {model_type} output: '{vlm_text_output[:100]}...'")
        action = "stop"  # Default to safe action
    
    if fast_mode:
        reasoning_snippet = f"Fast mode ({model_type}): {action.upper()}"
    else:
        reasoning_snippet = f"{model_type} reasoning: {vlm_text_output[:150]}..."
    
    logger.info(f"{model_type} output: '{vlm_text_output[:100]}...' -> Parsed action: {action}")
    return action, reasoning_snippet

@app.post("/switch_model")
async def switch_model(request: ModelSwitchRequest):
    """Switch between available VLM models"""
    global OLLAMA_VLM_MODEL_NAME, CURRENT_MODEL_TYPE, latest_data_for_gui
    
    if request.model not in AVAILABLE_MODELS:
        raise HTTPException(
            status_code=400, 
            detail=f"Invalid model. Available models: {list(AVAILABLE_MODELS.keys())}"
        )
    
    new_model_name = AVAILABLE_MODELS[request.model]
    
    try:
        # Check if the new model is available
        ollama_client.show(new_model_name)
        
        # Switch to new model
        OLLAMA_VLM_MODEL_NAME = new_model_name
        CURRENT_MODEL_TYPE = request.model
        latest_data_for_gui["current_model"] = request.model
        
        logger.info(f"Switched to model: {request.model} ({new_model_name})")
        
        return {
            "status": "success",
            "previous_model": CURRENT_MODEL_TYPE,
            "new_model": request.model,
            "model_name": new_model_name
        }
        
    except ollama.ResponseError as e:
        logger.error(f"Model '{new_model_name}' not available: {e}")
        raise HTTPException(
            status_code=404,
            detail=f"Model '{new_model_name}' not found. Run: ollama pull {new_model_name}"
        )

@app.get("/available_models")
async def get_available_models():
    """Get list of available VLM models"""
    model_status = {}
    for key, model_name in AVAILABLE_MODELS.items():
        try:
            ollama_client.show(model_name)
            model_status[key] = {"name": model_name, "available": True}
        except:
            model_status[key] = {"name": model_name, "available": False}
    
    return {
        "available_models": model_status,
        "current_model": CURRENT_MODEL_TYPE,
        "recommended": {
            "rtx_4060": "gemma3-4b",
            "rtx_4070_plus": "gemma3-12b",
            "lower_vram": "llava-7b"
        }
    }

@app.post("/process_image", response_model=RobotCommandResponse)
async def handle_image_processing(request_data: VLMProcessingRequest):
    global latest_data_for_gui
    latest_data_for_gui["error"] = None
    start_time = datetime.datetime.now()

    if not ollama_client:
        err_msg = "Ollama client is not available. Cannot process request."
        logger.error(err_msg)
        latest_data_for_gui["error"] = err_msg
        raise HTTPException(status_code=503, detail="Ollama service unavailable or model not found.")

    # Handle model switching if requested
    model_to_use = request_data.model if request_data.model in AVAILABLE_MODELS else CURRENT_MODEL_TYPE
    model_name = AVAILABLE_MODELS[model_to_use]
    
    # Extract mission from prompt or use default
    mission = "explore safely"
    if "find" in request_data.prompt.lower():
        mission = "find objects"
    elif "friend" in request_data.prompt.lower():
        mission = "find robot friends"
    elif "book" in request_data.prompt.lower():
        mission = "find books"
    
    # Use optimized prompt based on model type
    optimized_prompt = get_optimized_prompt(request_data.fast_mode, mission, model_to_use)
    
    logger.info(f"Processing image with {model_to_use} in {'FAST' if request_data.fast_mode else 'DETAILED'} mode")
    latest_data_for_gui["timestamp"] = start_time.isoformat()
    latest_data_for_gui["received_image_base64"] = request_data.image_base64
    latest_data_for_gui["prompt_sent"] = optimized_prompt
    latest_data_for_gui["fast_mode"] = request_data.fast_mode
    latest_data_for_gui["current_model"] = model_to_use

    try:
        image_bytes = base64.b64decode(request_data.image_base64)
    except base64.binascii.Error as e:
        err_msg = f"Invalid Base64 string for image: {e}"
        logger.error(err_msg)
        latest_data_for_gui["error"] = err_msg
        raise HTTPException(status_code=400, detail=err_msg)

    try:
        logger.info(f"Sending image to Ollama model: {model_name}...")
        
        # Performance optimization based on model type
        generation_params = {
            "model": model_name,
            "prompt": optimized_prompt,
            "images": [image_bytes],
            "stream": False
        }
        
        # Model-specific optimizations
        if model_to_use.startswith("gemma3"):
            # Gemma 3 optimizations
            generation_params.update({
                "options": {
                    "temperature": 0.1 if request_data.fast_mode else 0.3,
                    "top_p": 0.9,
                    "num_predict": 5 if request_data.fast_mode else 80,
                    "stop": ["<|im_end|>", "\n\n"] if request_data.fast_mode else None
                }
            })
        else:
            # LLaVA optimizations
            generation_params.update({
                "options": {
                    "temperature": 0.1 if request_data.fast_mode else 0.2,
                    "top_p": 0.9,
                    "num_predict": 10 if request_data.fast_mode else 100
                }
            })
        
        response_from_ollama = ollama_client.generate(**generation_params)
        
        processing_time = (datetime.datetime.now() - start_time).total_seconds()
        
        raw_vlm_text_output = response_from_ollama.get("response", "").strip()
        logger.info(f"Raw response from {model_to_use} ({processing_time:.2f}s): '{raw_vlm_text_output}'")
        latest_data_for_gui["vlm_raw_output"] = raw_vlm_text_output
        latest_data_for_gui["processing_time"] = processing_time

        if not raw_vlm_text_output:
            logger.warning(f"{model_to_use} returned an empty response.")
            action, reasoning = "stop", f"{model_to_use} returned empty response."
        else:
            action, reasoning = parse_vlm_output_to_command(raw_vlm_text_output, request_data.fast_mode, model_to_use)
        
        # Dynamic speed based on action and model performance
        current_speed = 0.5
        if action == "stop":
            current_speed = 0.0
        elif action == "forward":
            # Gemma 3 tends to be more confident, can go slightly faster
            current_speed = 0.35 if model_to_use.startswith("gemma3") else 0.3
            if request_data.fast_mode:
                current_speed += 0.05  # Slight boost for fast mode
        elif action in ["left", "right"]:
            current_speed = 0.45 if model_to_use.startswith("gemma3") else 0.4
            if request_data.fast_mode:
                current_speed += 0.05

        command_response = RobotCommandResponse(
            action=action,
            speed=current_speed,
            reasoning=reasoning,
            raw_vlm_output=raw_vlm_text_output,
            processing_time=processing_time,
            model_used=model_to_use
        )
        latest_data_for_gui["command_sent"] = command_response.model_dump()
        
        # Log performance metrics
        fps = 1.0 / processing_time if processing_time > 0 else 0
        logger.info(f"Processing complete: {action.upper()} (Speed: {current_speed}, FPS: {fps:.2f}, Model: {model_to_use})")
        
        return command_response

    except ollama.ResponseError as e:
        processing_time = (datetime.datetime.now() - start_time).total_seconds()
        err_msg = f"Ollama API error occurred: {e.status_code} - {e.error}"
        logger.error(err_msg)
        latest_data_for_gui["error"] = err_msg
        latest_data_for_gui["vlm_raw_output"] = f"Ollama Error: {e.error}"
        latest_data_for_gui["processing_time"] = processing_time
        latest_data_for_gui["command_sent"] = RobotCommandResponse(
            action="stop", reasoning=err_msg, raw_vlm_output=f"Ollama Error: {e.error}", 
            processing_time=processing_time, model_used=model_to_use
        ).model_dump()
        raise HTTPException(status_code=502, detail=err_msg)
    except Exception as e:
        processing_time = (datetime.datetime.now() - start_time).total_seconds()
        err_msg = f"An unexpected error occurred during VLM processing: {e}"
        logger.error(err_msg, exc_info=True)
        latest_data_for_gui["error"] = err_msg
        latest_data_for_gui["vlm_raw_output"] = "Unexpected server error."
        latest_data_for_gui["processing_time"] = processing_time
        latest_data_for_gui["command_sent"] = RobotCommandResponse(
            action="stop", reasoning=err_msg, raw_vlm_output="Server Error", 
            processing_time=processing_time, model_used=model_to_use
        ).model_dump()
        raise HTTPException(status_code=500, detail=err_msg)

# Enhanced mission endpoint with model awareness
@app.post("/set_mission")
async def set_mission(mission: str = "explore"):
    """Set robot mission to change behavior dynamically"""
    valid_missions = ["explore", "find_books", "find_friends", "navigate", "clean"]
    if mission not in valid_missions:
        raise HTTPException(status_code=400, detail=f"Invalid mission. Valid missions: {valid_missions}")
    
    latest_data_for_gui["current_mission"] = mission
    return {
        "status": "success", 
        "mission": mission, 
        "message": f"Robot mission set to: {mission}",
        "current_model": CURRENT_MODEL_TYPE
    }

# --- GUI Endpoints ---
@app.get("/", response_class=HTMLResponse)
async def serve_gui(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/gui_status")
async def get_gui_status():
    return latest_data_for_gui

# Enhanced performance monitoring with model information
@app.get("/performance")
async def get_performance_stats():
    """Get current performance statistics"""
    return {
        "model": OLLAMA_VLM_MODEL_NAME,
        "model_type": CURRENT_MODEL_TYPE,
        "last_processing_time": latest_data_for_gui.get("processing_time", 0),
        "estimated_fps": 1.0 / latest_data_for_gui.get("processing_time", 1) if latest_data_for_gui.get("processing_time", 0) > 0 else 0,
        "fast_mode_enabled": latest_data_for_gui.get("fast_mode", False),
        "current_mission": latest_data_for_gui.get("current_mission", "explore"),
        "available_models": list(AVAILABLE_MODELS.keys()),
        "recommended_for_hardware": {
            "rtx_4060": "gemma3-4b",
            "rtx_4070_plus": "gemma3-12b", 
            "lower_vram": "llava-7b"
        }
    }

# --- Main execution guard ---
if __name__ == "__main__":
    logger.info("Starting Enhanced Duckiebot VLM Bridge Server with Gemma 3 Support...")
    logger.info(f"Default Model: {CURRENT_MODEL_TYPE} ({OLLAMA_VLM_MODEL_NAME})")
    logger.info("Supported Models:")
    for key, model in AVAILABLE_MODELS.items():
        logger.info(f"  {key}: {model}")
    logger.info("Features: Fast Mode, Mission Control, Performance Monitoring, Multi-Model Support")
    uvicorn.run(app, host="0.0.0.0", port=5000)
