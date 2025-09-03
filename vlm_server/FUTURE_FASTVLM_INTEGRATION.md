# Future FastVLM-7B Integration Guide

## Overview
This document outlines how to integrate Apple's FastVLM-7B for improved performance while maintaining current Duckiebot functionality.

## Performance Benefits
- 85x faster Time-to-First-Token (TTFT)
- 3.4x smaller vision encoder
- Better vision-language understanding

## Implementation Steps

### 1. Environment Setup
```bash
# Create new FastVLM environment
conda create -n fastvlm python=3.10
conda activate fastvlm

# Clone FastVLM repository
git clone https://github.com/apple/ml-fastvlm.git
cd ml-fastvlm
pip install -e .

# Download model
huggingface-cli download apple/FastVLM-7B
```

### 2. Create FastVLM API Server
```python
# fastvlm_server.py
import torch
from PIL import Image
from transformers import AutoTokenizer, AutoModelForCausalLM
from flask import Flask, request, jsonify
import base64
import io

app = Flask(__name__)

# Load FastVLM-7B
MID = "apple/FastVLM-7B"
IMAGE_TOKEN_INDEX = -200

tok = AutoTokenizer.from_pretrained(MID, trust_remote_code=True)
model = AutoModelForCausalLM.from_pretrained(
    MID,
    torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
    device_map="auto",
    trust_remote_code=True,
)

@app.route('/v1/chat/completions', methods=['POST'])
def chat_completions():
    data = request.json
    
    # Extract prompt and image
    prompt = data['messages'][0]['content'][0]['text']
    image_data = data['messages'][0]['content'][1]['image_url']['url']
    
    # Decode base64 image
    image_bytes = base64.b64decode(image_data.split(',')[1])
    image = Image.open(io.BytesIO(image_bytes)).convert("RGB")
    
    # Build chat template
    messages = [{"role": "user", "content": f"<image>\n{prompt}"}]
    rendered = tok.apply_chat_template(messages, add_generation_prompt=True, tokenize=False)
    pre, post = rendered.split("<image>", 1)
    
    # Tokenize around image
    pre_ids = tok(pre, return_tensors="pt", add_special_tokens=False).input_ids
    post_ids = tok(post, return_tensors="pt", add_special_tokens=False).input_ids
    img_tok = torch.tensor([[IMAGE_TOKEN_INDEX]], dtype=pre_ids.dtype)
    input_ids = torch.cat([pre_ids, img_tok, post_ids], dim=1).to(model.device)
    attention_mask = torch.ones_like(input_ids, device=model.device)
    
    # Process image
    px = model.get_vision_tower().image_processor(images=image, return_tensors="pt")["pixel_values"]
    px = px.to(model.device, dtype=model.dtype)
    
    # Generate response
    with torch.no_grad():
        out = model.generate(
            inputs=input_ids,
            attention_mask=attention_mask,
            images=px,
            max_new_tokens=50,
            temperature=0.3,
        )
    
    response_text = tok.decode(out[0], skip_special_tokens=True)
    
    return jsonify({
        "choices": [{
            "message": {"content": response_text}
        }]
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
```

### 3. Update Configuration
```python
# Update vlm_server URLs to point to FastVLM server
FASTVLM_SERVER_URL = "http://localhost:8080"
```

### 4. Migration Strategy
1. **Phase 1**: Test FastVLM with current duck protection prompts
2. **Phase 2**: Benchmark performance vs current Qwen2.5-VL
3. **Phase 3**: Full migration if benefits justify complexity

## Considerations
- **Memory Requirements**: ~15GB GPU memory for FastVLM-7B
- **Setup Complexity**: More complex than current llama.cpp container
- **Maintenance**: Custom code vs standardized llama.cpp
- **Reliability**: New model vs proven Qwen2.5-VL system

## Decision Framework
Migrate to FastVLM-7B if:
- ✅ Current system has latency issues
- ✅ Need faster real-time responses
- ✅ Team comfortable with PyTorch deployment
- ✅ Have sufficient GPU memory (15GB+)

Stay with current system if:
- ✅ Performance is acceptable
- ✅ Reliability is priority
- ✅ Simple maintenance preferred
- ✅ Limited GPU memory (<15GB)



