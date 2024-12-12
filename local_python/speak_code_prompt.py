from openai import OpenAI
from mysecrets import api_key
from whispertotext import Speech2Text
import numpy as np
import os
import re
import subprocess

client = OpenAI(api_key=api_key)

# Audio Recording Settings
SAMPLE_RATE = 44100
CHANNELS = 1
MAX_FILE_SIZE_MB = 25
stt_prompt = "User is speaking a prompt for a LLM to be transcribed."
stt = Speech2Text(client, stt_prompt, SAMPLE_RATE, CHANNELS, MAX_FILE_SIZE_MB)

def generate_and_run_code():
    input_method = input("Choose input method - Type 'text' for text input or 'speech' for speech input: ").strip().lower()
    while input_method not in ["text", "speech"]:
        input_method = input("Invalid input. Please type 'text' or 'speech': ").strip().lower()

    if input_method == "text":
        user_prompt = input("Please enter your prompt: ").strip()
    elif input_method == "speech":
        while True:
            audio_directory = "recordings"
            if not os.path.exists(audio_directory):
                os.makedirs(audio_directory)
            audio_file_path = os.path.join(audio_directory, "user_prompt_audio.wav")
            stt.record_audio(audio_file_path)

            print(" Checking if audio file needs to be split...")
            chunks = stt.split_audio_if_needed(audio_file_path)

            print(" Transcribing audio...")
            user_prompt = stt.transcribe_audio_chunks(chunks)
            print(f" Transcription result: {user_prompt}")

            if stt.confirm_transcription(user_prompt) is True:
                # exit recording loop if user confirms accurate transcription
                break

    # Send the request to OpenAI API to generate Python code
    print(" Sending request to OpenAI API.")
    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {
                "role": "system",
                "content": (
                    "You are an expert Python programmer AI assistant that outputs only Python code without any explanation or extra text.\n"
                    "You only produce perfect, well formatted and commented code.\n\n"
                    "System Details and Constraints:\n"
                    "You do not have access to the primitives folder, you must redefine any functions you would like to use from the primitives directory"
                    "- All units are in radians for joint angles and meters for positions.\n"
                    "- Do not assume anything about the system that has not been explicitly stated.\n"
                    "- The environment is a ROS2-based system with the following available:\n"
                    "  * Action server: /simulated_arm_server_1/follow_joint_trajectory\n"
                    "    - Action interface: control_msgs/action/FollowJointTrajectory\n"
                    "  * Publisher: /simulated_arm_server1/gripper/control\n"
                    "    - Message type: std_msgs/msg/Int32MultiArray\n"
                    "  * Service: /simulated_arm_server1/get_current_pose\n"
                    "    - Service type: std_srvs/srv/Trigger\n"
                    "- Primitives are defined in `primitives/robot_primitives.py` and must be used.\n"
                    "- A reference format and approach is shown in `primitives/demo.py`. You must follow that style and approach.\n"
                    "- All code must be able to run standalone assuming:\n"
                    "  * The ROS2 environment is correctly sourced.\n"
                    "  * The packages (rclpy, control_msgs, std_msgs, std_srvs, myur) are installed and accessible.\n"
                    "  * The robot_primitives.py and demo.py files are in `primitives/` directory available in the Python path.\n\n"
                    "Error Handling and Behavior:\n"
                    "- If a service or action server is not available, gracefully handle it by printing an error or returning.\n"
                    "- Do not invent new hardware, sensors, or transformations not mentioned.\n"
                    "- Do not change units or coordinate frames.\n"
                    "- Only call and reference primitives as they have been described. If a primitive is not defined, do not invent it.\n\n"
                    "Formatting and Output:\n"
                    "- Produce only Python code as the response, no additional commentary or explanations.\n"
                    "- Double check that the imports match the described primitives and the formatting of `demo.py`.\n"
                    "- Ensure code runs standalone (i.e., includes `rclpy.init()` and a `main()` function when necessary) and follows best practices shown in `demo.py`.\n\n"
                    "Scenario Reference:\n"
                    "- The `demo.py` file can be referenced as a good solution format. For example, if asked to pick up an object on the ground to the right\n"
                    "  and place it 90 degrees rotated on a shelf to the left, follow the style of `demo.py` to set up action clients, publishers, services,\n"
                    "  and primitives usage.\n"
                )
            },
            {
                "role": "assistant",
                "content": (
                    "# This assistant message clarifies that `demo.py` provides a good example.\n"
                    "# The code should not include explanations outside of code blocks.\n"
                    "# The code must strictly follow the specified interfaces, units, and no assumptions.\n"
                )
            },
            {
                "role": "user",
                "content": f"Write a Python program that achieves the following prompt: {user_prompt}. Use the primitives and follow the formatting and approach shown in `demo.py`. Ensure the response is only Python code and can run standalone."
            }
        ]
    )
    print(" Received response from OpenAI API.")

    # Get the generated Python code
    raw_output = response.choices[0].message.content.strip()
    print(f" Raw output received: {raw_output[:100]}... (truncated)")

    # Regular expression to find code between ```python and ```
    code_pattern = r"```(?:python)?\n(.*?)\n```"
    match = re.search(code_pattern, raw_output, re.DOTALL)

    if match:
        code = match.group(1).strip()
        print(" Extracted code from formatted block.")
    else:
        if "import" in raw_output or "def" in raw_output or "class" in raw_output:
            code = raw_output
            print(" Assuming raw output is valid Python code.")
        else:
            print("The response doesn't contain valid Python code. Please try again.")
            return

    print("\nGenerated Code:\n")
    print(code)

    folder_name = "responses"
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
    file_path = os.path.join(folder_name, "generated_code.py")
    with open(file_path, "w") as file:
        file.write(code)
    print(f"\nSaved the code to '{file_path}'.")

    execute = input("Do you want to run this code? (yes/no): ").strip().lower()
    if execute == "yes":
        print("\nRunning the code as a separate program...\n")
        try:
            result = subprocess.run(
                ["python3", file_path],
                capture_output=True,
                text=True
            )
            if result.stdout:
                print("Output:\n", result.stdout)
            if result.stderr:
                print("Errors:\n", result.stderr)
        except Exception as e:
            print(f"Error while executing the program: {e}")


# Run the program
generate_and_run_code()