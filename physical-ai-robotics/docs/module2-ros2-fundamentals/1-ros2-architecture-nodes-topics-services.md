---
sidebar_position: 1
---

# ROS 2 Architecture, Nodes, Topics, and Services

## Overview
This chapter introduces the fundamental concepts of the Robot Operating System 2 (ROS 2). We will dissect its architecture and understand the core communication mechanisms: nodes, topics, and services. Think of this as learning the grammar of the language that robots use to talk to each other.

## Learning Outcomes
- Understand the distributed nature of the ROS 2 graph.
- Define what a ROS 2 Node is and its role.
- Differentiate between Topics (asynchronous, one-to-many) and Services (synchronous, one-to-one).
- Understand the concept of message types for structuring data.

## Real-life example
Imagine a restaurant kitchen. The "Chef" (a node) continuously "shouts out" orders as they come in (publishes to a "orders" topic). The "Line Cooks" (subscriber nodes) listen for these orders and start preparing the food. If a cook needs a specific ingredient, they might "ask" the "Pantry Manager" (a service server) for it and wait for a direct response (a service call).

## Technical explanation with diagrams
The ROS 2 Graph is a network of nodes that communicate with each other. The primary methods are Topics (for continuous data streams) and Services (for request/reply interactions).

```mermaid
graph TD
    subgraph ROS 2 Graph
        NodeA[Camera Node]
        NodeB[Image Processor Node]
        NodeC[Obstacle Detection Node]
        ServiceD[/get_robot_status]
    end

    NodeA -- "Image Data (Topic: /image_raw)" --> NodeB
    NodeB -- "Processed Image (Topic: /image_proc)" --> NodeC
    NodeC -- "Obstacles (Topic: /obstacles)" --> OtherNodes[...]
    
    ControllerNode[Controller Node] -. "Request" .-> ServiceD
    ServiceD -. "Response" .-> ControllerNode
    
```
*Figure 1: A simple ROS 2 graph showing nodes, topics, and a service.*

## Code examples (Python/rclpy)
```python
# Placeholder for a simple ROS 2 publisher using rclpy

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Glossary
- **ROS (Robot Operating System)**: A flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.
- **Node**: An independent, executable process in a ROS 2 system that performs some computation.
- **Topic**: A named bus over which nodes exchange messages. Topics are used for asynchronous, publish/subscribe communication.
- **Service**: A request/reply communication method where one node (the client) sends a request to another node (the server) and waits for a response.

## Quiz Questions
1. What is the primary role of a ROS 2 node?
    a) To store data.
    b) To perform computation and communication.
    c) To define message structures.
    d) To launch the entire system.

2. You need to continuously stream camera images to multiple other nodes. Should you use a topic or a service?
    a) Service, because the data is important.
    b) Topic, because it supports one-to-many, asynchronous communication.
    c) Either would work.
    d) Neither, ROS 2 cannot handle images.

3. What does it mean for a service to be "synchronous"?

4. Can two nodes publish to the same topic?

5. What is a ROS 2 "message" in the context of a topic?
