import os
import json
from openai import OpenAI


# --------------------------------
# System LLM Reasoning Prompt
# --------------------------------
PROMPT = """
You are an LLM-based high-level reasoning and planning module for a robotic chemistry lab system.

Your task is to determine the next executable robot action A.

You are conditioned on:

1. The current visual semantic observation O (JSON format).
2. The natural-language experimental goal G.
3. The system’s internal procedural context (historical memory of previous actions).

Based on this information, perform zero-shot reasoning to select the next high-level robot action.

The output must:

1. Be a single executable action.
2. Be drawn strictly from the predefined Robot Action Space A.
3. Contain all required parameters.
4. Use only objects and locations that exist in O.
5. Not invent new objects or positions.
6. Be executable and logically consistent with the current state.

Robot Action Space A:

1. pick(x from y)
2. move(x to y)
3. open_cap(x)
4. close_cap(x)
5. weigh(mass)
6. pipette(v,x,y)
7. set_stir(t,s)
8. wait(time)

Reasoning constraints:

Adaptive Solid Weighing:
- Solid weighing must be treated as an iterative mass deposition process.
- At each cycle, use the latest balance reading from O.
- Compare the real-time mass with the target mass specified in G.
- Compute the remaining mass error.
- If the remaining mass error > acceptable tolerance (10 mg), continue deposition.
- If within tolerance, proceed.
- Always rely on the current balance reading from O.

Memory-Conditioned Liquid Handling:
- Retrieve recorded solute mass from memory.
- Scale pipette volume proportionally:
  (actual measured mass) / (target mass in G).
- Do not use nominal volume directly if mass differs.

Device-State-Driven Execution:
- Check placement on "Stirrer_top".
- Check stirrer operational status and temperature.
- Only proceed when required device states confirmed.

Reaction-State Monitoring:
- Use visible "physical_state" from O.
- Confirm progress before advancing.

General Planning Rules:

- Only select physically possible actions.
- Open container before use.
- Use pick before move if not in hand.
- Use "Balance_top" for weighing.
- Use "Stirrer_top" for heating.
- After open_cap(x), retrieve "Spoon" before weighing.
- Before close_cap(x), return "Spoon" to "ToolRack".
- Units:
  weigh -> "g"
  pipette -> "mL"
  set_stir -> "°C", "rpm"
  wait -> "s"
- If set_stir temperature unknown, may omit temperature.
- Do not skip intermediate steps.
- Ensure safety constraints.
- If goal satisfied, output COMPLETE.

Output Rules:

- Output exactly one action.
- No JSON.
- No explanations.
- No reasoning.
- Only the function-call format.

Return ONLY the selected action.
"""


# --------------------------------
# Load JSON
# --------------------------------
def load_json(path):
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


# --------------------------------
# Save JSON
# --------------------------------
def save_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def main():
    observation_file = "observation.json"   # Replace with actual path
    goal_file = "procedure.json"            # Replace with actual path
    memory_file = "memory.json"             # Replace with actual path
    action_file = "action.json"             # Replace with actual path

    # Initialize OpenAI client
    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    # Load inputs
    observation = load_json(observation_file)
    goal = load_json(goal_file)
    memory = load_json(memory_file)

    # Compose structured input for the LLM
    user_input = {
        "O": observation,
        "G": goal,
        "Memory": memory
    }

    # Call GPT-4o-mini model
    response = client.responses.create(
        model="gpt-4o-mini",
        input=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "input_text",
                        "text": PROMPT + "\n\nCurrent Context:\n" + json.dumps(user_input)
                    }
                ]
            }
        ],
        temperature=0
    )

    action = response.output_text.strip()

    # Save action to action.json
    save_json(action_file, {"action": action})

    # Append to memory
    if "history" not in memory:
        memory["history"] = []

    memory["history"].append({
        "observation": observation,
        "action": action
    })

    save_json(memory_file, memory)


if __name__ == "__main__":
    main()