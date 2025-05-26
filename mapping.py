import numpy as np
import cv2
import math

# Constants
MAP_SIZE = 1000
WORLD_SIZE = 2  # World size in meters (e.g., 2x2m); adjust to match your Webots world

class Mapping:
    def __init__(self, mapper):
        self.mapper = mapper
        self.first_update = True
        self.prev_position = None
        self.scale = MAP_SIZE / WORLD_SIZE  # Grid units per meter
        self.obstacle_positions = []  # Shared list for slave-reported obstacles

    def update_position(self):
        gps_values = self.mapper.gps.getValues()
        if gps_values and len(gps_values) >= 3:
            # Map Webots GPS (x, z) to grid (x_grid, y_grid)
            x_world = gps_values[0]  # Webots x-axis
            z_world = gps_values[1]  # Webots z-axis
            # Scale and center the coordinates on the map
            x_grid = int((x_world * self.scale) + (MAP_SIZE / 2))
            y_grid = int((-z_world * self.scale) + (MAP_SIZE / 2))  # FIXED: Negate z_world
            # Clamp to map boundaries
            x_grid = max(0, min(MAP_SIZE - 1, x_grid))
            y_grid = max(0, min(MAP_SIZE - 1, y_grid))
            self.mapper.position = [x_grid, y_grid]
            self.update_visited_map(x_grid, y_grid)
            self.first_update = False

    def update_visited_map(self, x_grid, y_grid):
        if hasattr(self.mapper, 'visited_map'):
            current_pos = (x_grid, y_grid)
            if self.prev_position is not None:
                # Draw thicker line for robot's own path
                cv2.line(self.mapper.visited_map, self.prev_position, current_pos, [255, 255, 255], thickness=10)
            else:
                # Larger starting point
                cv2.circle(self.mapper.visited_map, current_pos, 10, [255, 255, 255], -1)
            self.prev_position = current_pos

    def update_map_from_proximity(self):
        if not self.mapper.is_master:
            return
        readings = {'front': 1000, 'left': 1000, 'right': 1000}
        if self.mapper.proximity_sensors:
            front = [0, 7]
            left = [4, 5, 6]
            right = [1, 2, 3]
            readings['front'] = min(self.mapper.proximity_sensors[i].getValue() for i in front)
            readings['left'] = min(self.mapper.proximity_sensors[i].getValue() for i in left)
            readings['right'] = min(self.mapper.proximity_sensors[i].getValue() for i in right)
        if readings['front'] < 100:
            x_grid = int(self.mapper.position[0] + 10 * np.cos(self.mapper.orientation))
            y_grid = int(self.mapper.position[1] - 10 * np.sin(self.mapper.orientation))  # FIXED: Negate sin
            x_grid = max(0, min(MAP_SIZE - 1, x_grid))
            y_grid = max(0, min(MAP_SIZE - 1, y_grid))
            if 0 <= x_grid < MAP_SIZE and 0 <= y_grid < MAP_SIZE:
                cv2.circle(self.mapper.visited_map, (x_grid, y_grid), 5, [128, 128, 128], -1)

    def update_map_from_slave_obstacles(self):
        if not self.mapper.is_master:
            return
        # Process all reported obstacle positions
        while self.obstacle_positions:
            x_grid, y_grid = self.obstacle_positions.pop(0)  # Remove and process first position
            if 0 <= x_grid < MAP_SIZE and 0 <= y_grid < MAP_SIZE:
                cv2.circle(self.mapper.visited_map, (x_grid, y_grid), 5, [128, 128, 128], -1)

    def update_map_from_gps(self, robot_id, position, orientation):
        if not self.mapper.is_master or robot_id == self.mapper.robot_id:
            return
        x_grid, y_grid = int(position[0]), int(position[1])
        x_grid = max(0, min(MAP_SIZE - 1, x_grid))
        y_grid = max(0, min(MAP_SIZE - 1, y_grid))
        # Mark position for other robots
        if 0 <= x_grid < MAP_SIZE and 0 <= y_grid < MAP_SIZE:
            cv2.circle(self.mapper.visited_map, (x_grid, y_grid), 10, [255, 255, 255], -1)