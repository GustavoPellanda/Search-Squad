# Search-Squad
Final project for the Mobile Robotics course, offered by the Graduate Program in Applied Computing (PPGCA) at UTFPR Curitiba campus, during the second semester of 2026.

A ROS 2-based multi-robot exploration and mapping project, featuring autonomous navigation, collaborative mapping, and coordinated task allocation in simulated environments.

# Project Overview

The goal of this project is to coordinate a fleet of robots to explore and map a large indoor environment, such as an office building or a shopping mall. The idea was inspired by the work of first responders, such as firefighters, SWAT teams, and emergency medical teams, who are often faced with the challenge of entering an unknown building through a main entrance and systematically covering its entire area, searching for victims, suspects, or potential hazards.

By developing this project, my main objective is to study how to coordinate multiple robots simultaneously, each with different tasks, environment perception strategies, and navigation algorithms, working together towards a common goal: exploring a large environment and combining their individual contributions into a single, shared map.

The proposed system consists of a **Scout**, responsible for navigating corridors, and a group of **Explorers**, responsible for exploring and mapping individual rooms.

As the Scout moves through the main corridor of a building, it progressively maps its surroundings using LiDAR data, identifying walls, obstacles, and openings that may represent entrances to rooms. Whenever a new entrance is detected, its coordinates are registered by a centralized coordination system. This is where the Explorers come into action.

The coordination system keeps track of the discovered room entrances and assigns exploration tasks to the available Explorers. Each Explorer receives a destination and uses autonomous navigation to travel through the previously mapped corridor until it reaches its assigned room. Once inside, it switches to a reactive navigation strategy to explore the room, avoid obstacles, and contribute new information to the shared map.
After completing its exploration task, the Explorer returns to the room entrance and becomes available for a new assignment. This process continues until all detected rooms have been explored.

The project brings together several fundamental concepts of Mobile Robotics:

- **Environment perception:** Using LiDAR sensors to detect walls, obstacles, and free space.
- **Reactive navigation:** Allowing the Explorers to navigate inside rooms and avoid obstacles based on real-time sensor readings.
- **Odometry:** Estimating robot movement and position to support localization, navigation, and mapping.
- **Kinematics:** Applying mathematical models to calculate and control the motion of each robot.
- **Coordinate transformations:** Converting positions and sensor measurements between different reference frames, allowing robots to interpret and share spatial information consistently.
- **Fuzzy control:** Implementing a fuzzy logic controller to guide the Scout along corridors based on its LiDAR measurements.
- **SLAM (Simultaneous Localization and Mapping):** Building and updating an environment map while estimating robot poses as they navigate.
- **Autonomous Navigation:** Planning and following paths through a mapped environment, allowing Explorers to autonomously reach their assigned destinations while avoiding obstacles.

On top of these concepts, the project introduces the additional challenge of developing a centralized multi-robot coordination system, capable of managing different robot behaviors, distributing tasks, and maintaining a shared representation of the environment.

### Minimal Proof of Concept

The initial goal is to demonstrate the system in a simple simulated environment consisting of a straight corridor with four rooms.
A single Scout navigates through the corridor, identifies the room entrances, and provides the necessary mapping information. Two Explorers are then assigned to the detected rooms, navigating to their destinations and performing their exploration tasks.
This scenario is intended to validate the main components of the system: corridor navigation, room detection, task allocation, autonomous navigation, and collaborative mapping.

### Desired Complete System

The long-term goal is to extend the system to more complex environments, with multiple interconnected corridors, several Scouts, and a larger fleet of Explorers.
Ideally, the robots would distribute themselves throughout the environment, dynamically receiving new assignments and cooperating to explore and map the entire area as efficiently as possible.
