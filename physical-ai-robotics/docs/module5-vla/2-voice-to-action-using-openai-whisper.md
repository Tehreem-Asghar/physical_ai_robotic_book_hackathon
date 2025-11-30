---
sidebar_position: 2
---

# Voice-to-Action using OpenAI Whisper

## Overview
This chapter demonstrates how to build a voice interface for a robot using OpenAI's Whisper model. Whisper is a powerful automatic speech recognition (ASR) system that can transcribe spoken language into text with high accuracy. We will learn how to capture audio, send it to Whisper for transcription, and use the resulting text as a command for the robot.

## Learning Outcomes
- Understand the role of Automatic Speech Recognition (ASR) in robotics.
- Learn about OpenAI Whisper and its capabilities.
- Set up a system to capture microphone audio and send it to the Whisper API.
- Integrate the transcribed text with the LLM-based planning system from the previous chapter.

## Real-life example
Instead of typing a command, a user simply speaks to their robot: "Robot, please bring me the red ball from the table." A microphone on the robot captures the audio. The audio is sent to Whisper, which transcribes it into the text string "Robot, please bring me the red ball from the table." This text is then fed to the robot's LLM planner, which generates the action plan to fulfill the request.

## Technical explanation with diagrams
The voice-to-action pipeline involves several steps: capturing audio, transcribing it to text (ASR), and then processing that text to generate a robot action. Each step can be a ROS 2 node, creating a modular and extensible system.

```mermaid
graph TD
    A[User Speaks] --> B(Microphone)
    B -- "Audio Stream" --> C[Audio Capture Node (ROS)]
    C -- "Audio Data" --> D{Whisper ASR Node}
    D -- "API Call" --> E(OpenAI Whisper Service)
    E -- "Transcribed Text" --> D
    D -- "Text Command (ROS Topic)" --> F(LLM Planner Node)
    F --> G[Robot Action]
```
*Figure 1: A voice-to-action pipeline using Whisper in a ROS 2 system.*

## Code examples (Conceptual Python)
```python
# Conceptual placeholder for a Whisper ASR ROS 2 node

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray

import openai
import numpy as np

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        # Subscribes to an audio topic
        self.audio_sub = self.create_subscription(
            Int16MultiArray, 'audio_stream', self.audio_callback, 10
        )
        # Publishes the transcribed text
        self.text_pub = self.create_publisher(String, 'transcribed_text', 10)
        
        self.audio_buffer = []

    def audio_callback(self, msg):
        # Buffer audio until we detect silence
        self.audio_buffer.extend(msg.data)
        
        # This is a simplified silence detection
        if self.is_silence(msg.data):
            if len(self.audio_buffer) > 22050: # If we have at least 1s of audio
                self.get_logger().info('Silence detected, transcribing...')
                
                # Convert buffer to numpy array and send to Whisper
                audio_np = np.array(self.audio_buffer, dtype=np.int16)
                # In a real app, you'd save to a temp file or stream it
                # For this example, we'll just conceptually call it
                # transcribed_text = self.transcribe_with_whisper(audio_np)
                
                transcribed_text = "This is a simulated transcription." # Fake response
                
                # Publish the result
                text_msg = String()
                text_msg.data = transcribed_text
                self.text_pub.publish(text_msg)
                self.get_logger().info(f'Published text: "{transcribed_text}"')

            self.audio_buffer = [] # Clear buffer

    def is_silence(self, audio_chunk):
        # Placeholder for actual silence detection logic
        return np.mean(np.abs(audio_chunk)) < 100

    def transcribe_with_whisper(self, audio_np):
        # This would involve saving the numpy array to a WAV file
        # and then calling the OpenAI API.
        # e.g., audio_file = open("temp.wav", "rb")
        # transcript = openai.Audio.transcribe("whisper-1", audio_file)
        # return transcript['text']
        pass
```

## Glossary
- **ASR (Automatic Speech Recognition)**: A technology that enables a computer to identify and process human speech and convert it into a written format.
- **OpenAI Whisper**: A state-of-the-art ASR model from OpenAI, known for its high accuracy across a wide range of languages and accents.
- **Endpointing**: The process of detecting the beginning and end of speech in an audio stream, often by looking for silence.

## Quiz Questions
1. What does ASR stand for?
    a) Automated System reboot
    b) Audio Signal rectification
    c) Automatic Speech Recognition
    d) Action Service Request

2. What role does OpenAI Whisper play in a voice-controlled robot?
    a) It decides what action the robot should take.
    b) It converts the user's spoken words into text.
    c) It generates the robot's voice response.
    d) It moves the robot's motors.

3. In the voice-to-action pipeline diagram, what happens immediately after the ASR node transcribes the audio?

4. What is "endpointing" and why is it important for a voice interface?

5. What kind of ROS 2 message would be suitable for publishing raw audio data from a microphone?
