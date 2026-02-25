import os
import json
import base64
from openai import OpenAI


# --------------------------------
# System VLM Perception Prompt
# --------------------------------
PROMPT = """
You are a Vision-Language Model (VLM) perception module for a robotic chemistry lab system.

Your task is to transform raw visual input into a structured semantic observation in JSON format.

The output must:

1. Be valid JSON.
2. Contain exactly three top-level keys:
- "TOOLS"
- "EQUIPMENT"
- "CHEMICALS"
3. Each detected object must be described with structured attributes.
4. For each object, include:
- identity (as key name)
- location (relative spatial description)
- physical_state
- status
- numerical_reading (if visible)
5. Do NOT hallucinate invisible values.
6. If a value is not visible, write: "unknown".
7. Use consistent location descriptions such as:
- "Table_left"
- "Table_center"
- "Table_right"
- "SolidRack_x"
- "LiquidRack_x"
- "ToolRack_x"
- "Balance_top"
- "Stirrer_top"
- "in_LeftHand"
- "in_RightHand"

Additional scene prior:

The visual input is a chemistry laboratory tabletop scene containing:

- One "Balance"
- One "Stirrer"
- Three fixed racks:
  - "ToolRack"
  - "SolidRack"
  - "LiquidRack"

The "ToolRack" has two fixed tool positions:
- "ToolRack_1" holds "Spoon"
- "ToolRack_2" holds "Pipette"

Interpretation rules:

- The "Balance" may have a digital numerical reading.
- The "Stirrer" may have heating and stirring status.
- Bottles on "SolidRack" contain solid chemicals.
- Bottles on "LiquidRack" contain liquid chemicals.
- Identify chemicals using visible labels only.
- If container cap appears closed, "status" = "closed".
- If container cap appears open, "status" = "open".
- If display is readable, extract exact numeric value as "reading".
- If powered state unclear, set "status" = "unknown".
- All objects must include all required fields even if values are "unknown".

For "EQUIPMENT":
- "status" indicates "on" or "off".

For "CHEMICALS":
- "type" must be "solid" or "liquid".
- "status" indicates whether cap appears "open" or "closed".
- "physical_state" describes visible color or appearance.

Output format:

{
  "TOOLS": {
    "ObjectName": {
      "location": ""
    }
  },
  "EQUIPMENT": {
    "ObjectName": {
      "status": "",
      "reading": "",
      "location": ""
    }
  },
  "CHEMICALS": {
    "ChemicalName": {
      "type": "",
      "status": "",
      "location": "",
      "physical_state": ""
    }
  }
}

Return ONLY the JSON object.
Do not include explanations.
Do not include markdown formatting.
Do not add comments.
"""


# --------------------------------
# Encode local image as base64
# --------------------------------
def encode_image(image_path):
    with open(image_path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")


def main():
    # Path to the local high-resolution RGB image (5472x3648)
    image_path = "image.jpg"  # Replace with actual image path
    output_file = "observation.json"

    # Initialize OpenAI client using environment variable OPENAI_API_KEY
    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    # Encode image
    base64_image = encode_image(image_path)

    # Call GPT-4o Vision model
    response = client.responses.create(
        model="gpt-4o",
        input=[
            {
                "role": "user",
                "content": [
                    {"type": "input_text", "text": PROMPT},
                    {
                        "type": "input_image",
                        "image_url": f"data:image/jpeg;base64,{base64_image}",
                        "detail": "high"
                    }
                ]
            }
        ],
        temperature=0
    )

    # Extract model output text (expected to be pure JSON)
    result_text = response.output_text

    # Parse JSON to ensure validity
    result_json = json.loads(result_text)

    # Save structured observation to file
    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(result_json, f, indent=2, ensure_ascii=False)

    print("Structured observation saved to:", output_file)


if __name__ == "__main__":
    main()