# E-puck-explorers
A Python-based multi-robot system for collaborative mapping and surveillance using Webots simulation, This project implements a swarm of autonomous robots that perform Simultaneous Localization and Mapping (SLAM) and real-time environmental monitoring.

## Features
- **Multi-Robot Coordination**: Master-slave architecture with networked communication for synchronized exploration.
- **Vision-Based Surveillance**: YOLOv8 for detecting and mapping obstacles and objects.
- **SLAM Implementation**: Frontier-based exploration with coverage tracking for 2D occupancy grid mapping.
- **Obstacle Avoidance**: Proximity sensor-driven navigation with stuck recovery.
- **Real-Time Visualization**: Dynamic map display with robot paths, FOV, and coverage percentage.

## Requirements
- Python 3.8+
- Webots R2023b or later
- Dependencies (see `requirements.txt`):
