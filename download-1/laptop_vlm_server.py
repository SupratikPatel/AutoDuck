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
OLLAMA_VLM_MODEL_NAME = "llava"
OLLAMA_API_HOST = None 

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- Pydantic Models ---
class VLMProcessingRequest(BaseModel):
    image_base64: str
    prompt: str = "Analyze this robot's camera view. What is the safest and most logical next action for a small robot to explore the area? Choose one: FORWARD, LEFT, RIGHT, or STOP."

class RobotCommandResponse(BaseModel):
    action: str
    speed: float = 0.5
    reasoning: str = ""
    raw_vlm_output: str = ""

# --- FastAPI Application and Templating ---
app = FastAPI(title="Duckiebot VLM Processing Server", version="0.2.0")

# Path to the templates directory (assuming it's in the same directory as this script)
templates = Jinja2Templates(directory="templates")

# --- In-memory store for the latest data for GUI ---
latest_data_for_gui = {
    "timestamp": None,
    "received_image_base64": None,
    "prompt_sent": None,
    "vlm_raw_output": None,
    "command_sent": None, 
    "error": None
}

# --- Ollama Client Initialization ---
try:
    ollama_client = ollama.Client(host=OLLAMA_API_HOST)
    logger.info(f"Attempting to connect to Ollama and check for model: '{OLLAMA_VLM_MODEL_NAME}'...")
    ollama_client.show(OLLAMA_VLM_MODEL_NAME)
    logger.info(f"Successfully connected to Ollama. Model '{OLLAMA_VLM_MODEL_NAME}' is available.")
except ollama.ResponseError as e:
    logger.error(f"Ollama model '{OLLAMA_VLM_MODEL_NAME}' not found or Ollama service error: {e}")
    logger.error(f"Please ensure Ollama is running and you have pulled the model (e.g., `ollama pull {OLLAMA_VLM_MODEL_NAME}`).")
    ollama_client = None
except Exception as e:
    logger.error(f"Failed to initialize or connect to Ollama client: {e}")
    ollama_client = None

def parse_vlm_output_to_command(vlm_text_output: str) -> tuple[str, str]:
    processed_text = vlm_text_output.upper()
    action = "unknown"
    if "STOP" in processed_text or "HALT" in processed_text:
        action = "stop"
    elif "FORWARD" in processed_text or "STRAIGHT" in processed_text:
        action = "forward"
    elif "LEFT" in processed_text:
        action = "left"
    elif "RIGHT" in processed_text:
        action = "right"
    else:
        logger.warning(f"Could not parse a clear command from VLM output: '{vlm_text_output}'")
    reasoning_snippet = f"VLM suggested: {vlm_text_output[:150]}"
    logger.info(f"VLM output: "{vlm_text_output[:100]}..." -> Parsed action: {action}")
    return action, reasoning_snippet

@app.post("/process_image", response_model=RobotCommandResponse)
async def handle_image_processing(request_data: VLMProcessingRequest):
    global latest_data_for_gui
    latest_data_for_gui["error"] = None

    if not ollama_client:
        err_msg = "Ollama client is not available. Cannot process request."
        logger.error(err_msg)
        latest_data_for_gui["error"] = err_msg
        raise HTTPException(status_code=503, detail="Ollama service unavailable or model not found.")

    logger.info(f"Received image for processing. Prompt: "{request_data.prompt}"")
    latest_data_for_gui["timestamp"] = datetime.datetime.now().isoformat()
    latest_data_for_gui["received_image_base64"] = request_data.image_base64
    latest_data_for_gui["prompt_sent"] = request_data.prompt

    try:
        image_bytes = base64.b64decode(request_data.image_base64)
    except base64.binascii.Error as e:
        err_msg = f"Invalid Base64 string for image: {e}"
        logger.error(err_msg)
        latest_data_for_gui["error"] = err_msg
        raise HTTPException(status_code=400, detail=err_msg)
    except Exception as e:
        err_msg = f"Error decoding or verifying image: {e}"
        logger.error(err_msg)
        latest_data_for_gui["error"] = err_msg
        raise HTTPException(status_code=400, detail=err_msg)

    try:
        logger.info(f"Sending image and prompt to Ollama model: {OLLAMA_VLM_MODEL_NAME}...")
        response_from_ollama = ollama_client.generate(
            model=OLLAMA_VLM_MODEL_NAME,
            prompt=request_data.prompt,
            images=[image_bytes],
            stream=False
        )
        
        raw_vlm_text_output = response_from_ollama.get("response", "").strip()
        logger.info(f"Raw response from Ollama VLM: "{raw_vlm_text_output}"")
        latest_data_for_gui["vlm_raw_output"] = raw_vlm_text_output

        if not raw_vlm_text_output:
            logger.warning("Ollama VLM returned an empty response.")
            action, reasoning = "stop", "VLM returned empty response."
        else:
            action, reasoning = parse_vlm_output_to_command(raw_vlm_text_output)
        
        current_speed = 0.5
        if action == "stop" or action == "unknown":
            current_speed = 0.0

        command_response = RobotCommandResponse(
            action=action,
            speed=current_speed,
            reasoning=reasoning,
            raw_vlm_output=raw_vlm_text_output
        )
        latest_data_for_gui["command_sent"] = command_response.model_dump()
        return command_response

    except ollama.ResponseError as e:
        err_msg = f"Ollama API error occurred: {e.status_code} - {e.error}"
        logger.error(err_msg)
        latest_data_for_gui["error"] = err_msg
        latest_data_for_gui["vlm_raw_output"] = f"Ollama Error: {e.error}"
        latest_data_for_gui["command_sent"] = RobotCommandResponse(action="stop", reasoning=err_msg, raw_vlm_output=f"Ollama Error: {e.error}").model_dump()
        raise HTTPException(status_code=502, detail=err_msg)
    except Exception as e:
        err_msg = f"An unexpected error occurred during VLM processing: {e}"
        logger.error(err_msg, exc_info=True)
        latest_data_for_gui["error"] = err_msg
        latest_data_for_gui["vlm_raw_output"] = "Unexpected server error."
        latest_data_for_gui["command_sent"] = RobotCommandResponse(action="stop", reasoning=err_msg, raw_vlm_output="Server Error").model_dump()
        raise HTTPException(status_code=500, detail=err_msg)

# --- GUI Endpoints ---
@app.get("/", response_class=HTMLResponse)
async def serve_gui(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/gui_status")
async def get_gui_status():
    return latest_data_for_gui

# --- Main execution guard ---
if __name__ == "__main__":
    logger.info("Starting Duckiebot VLM Bridge Server with GUI...")
    uvicorn.run(app, host="0.0.0.0", port=5000)
