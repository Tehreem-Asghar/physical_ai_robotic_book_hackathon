---
sidebar_position: 4
---

# Multi-modal Interaction: Speech, Gesture, Vision

## Overview
This chapter expands our robot's interaction capabilities beyond just speech. We'll explore how to create a multi-modal interface by combining language with other inputs, like gesture recognition and computer vision. This allows for richer, more natural, and less ambiguous communication with the robot.

## Learning Outcomes
- Understand the concept of multi-modal interaction.
- Learn how gesture recognition can augment voice commands.
- See how vision can be used to "ground" language (e.g., "pick up *that* one").
- Discuss strategies for fusing information from different modalities.

## Real-life example
A user in a workshop points to a specific screwdriver on a messy workbench and says, "Hand me that." A simple voice-only system would be confused. A multi-modal system, however, can combine the inputs. It uses Whisper to transcribe "Hand me that." It uses a gesture recognition model to detect the pointing gesture and its direction. It uses a vision model to see all the objects along the pointing vector. By fusing these, it correctly deduces that "that" refers to the screwdriver and can execute the command.

## Technical explanation with diagrams
Fusing data from multiple modalities is a key challenge. A central "fusion" node can be used to collect inputs from speech, vision, and gesture recognition systems. This node's job is to associate related inputs (e.g., a pointing gesture and a noun phrase) to create a single, unambiguous command for the cognitive planner.

```mermaid
graph TD
    A[Speech Recognition] -- Text: "pick up that red block" --> D
    B[Gesture Recognition] -- Vector: Pointing at a location --> D
    C[Object Detection] -- List of objects and their locations --> D
    
    subgraph "Multi-modal Fusion Node"
        D{Fusion Logic}
    end
    
    D -- "Disambiguated Command" --> E(Cognitive Planner)

    subgraph "Fusion Logic Example"
      direction LR
      D1("that") -- "associates with" --> D2(Pointing Gesture)
      D2 -- "intersects with" --> D3(Red Block's Location)
      D3 -- "resolves to" --> D4(Object ID: block_123)
      D4 -- "creates command" --> D5(pickup('block_123'))
    end
```
*Figure 1: Fusing multiple modalities to understand a command.*

## Code examples (Conceptual Python)
```python
# Conceptual placeholder for a multi-modal fusion node

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        # ...
        self.last_speech_command = None
        self.last_gesture = None
        self.detected_objects = []
        
        # Subscribe to inputs from other nodes
        self.create_subscription(String, 'speech_command', self.speech_cb, 10)
        self.create_subscription(Gesture, 'gesture_detected', self.gesture_cb, 10)
        self.create_subscription(ObjectList, 'detected_objects', self.objects_cb, 10)
        
        # Publisher for the final command
        self.command_pub = self.create_publisher(String, 'fused_command', 10)

    def speech_cb(self, msg):
        self.last_speech_command = msg.data
        self.try_to_fuse()

    def gesture_cb(self, msg):
        self.last_gesture = msg
        self.try_to_fuse()

    def objects_cb(self, msg):
        self.detected_objects = msg.objects

    def try_to_fuse(self):
        """Attempt to combine the latest inputs into a single command."""
        
        # This is where the core fusion logic would go.
        # It's a complex AI problem in itself.
        
        if self.last_speech_command and "that" in self.last_speech_command and self.last_gesture:
            self.get_logger().info("Fusing 'that' command with gesture...")
            
            # Find which object the gesture is pointing at
            target_object = self.find_object_from_gesture(self.last_gesture, self.detected_objects)
            
            if target_object:
                # Replace "that" with the object's ID
                fused_command = self.last_speech_command.replace("that", target_object.name)
                
                # Publish the new, specific command
                cmd_msg = String()
                cmd_msg.data = fused_command
                self.command_pub.publish(cmd_msg)
                
                # Clear the inputs so we don't re-trigger
                self.last_speech_command = None
                self.last_gesture = None

    def find_object_from_gesture(self, gesture, objects):
        # Placeholder for complex geometric calculation
        # It would project the gesture vector into the world
        # and find the closest object.
        if objects:
            return objects[0] # Just return the first object for demo
        return None
```

## Glossary
- **Multi-modal Interaction**: A type of human-computer interaction that uses multiple modes of input (like speech, gesture, gaze, etc.) to communicate.
- **Modality**: A channel of input or output, such as vision, hearing, or touch.
- **Gesture Recognition**: The process of interpreting human gestures (e.g., pointing, waving) using computer vision.
- **Data Fusion**: The process of combining data from multiple sources to create a more accurate and complete understanding than can be achieved from a single source.

## Quiz Questions
1. What is meant by "multi-modal interaction"?
    a) A robot that can perform many different tasks.
    b) Using multiple forms of input, like speech and gesture, to communicate.
    c) A robot that can speak multiple languages.
    d) A simulation with many different robot models.

2. Why is a multi-modal interface often less ambiguous than a voice-only interface?
    a) Because it's faster.
    b) Because it uses less battery.
    c) Because physical gestures (like pointing) can clarify vague words (like "that" or "it").
    d) Because it looks cooler.

3. In the diagram, what is the role of the "Fusion Logic"?

4. Give an example of a command that would be difficult for a voice-only system but easy for a multi-modal system.

5. What are some input "modalities" a robot might use? Name at least three.
