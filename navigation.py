import random
import math
import time
import cv2

# Constants
MAX_SPEED = 6.28
OBSTACLE_THRESHOLD = 100
STUCK_THRESHOLD = 5
BACKUP_TIME = 20
TURN_TIME = 5  # NEW: Time to turn approximately 90 degrees
MAP_SIZE = 1000

class Navigation:
    def __init__(self, mapper):
        self.mapper = mapper
        self.stuck_counter = 0
        self.is_backing_up = False
        self.backup_timer = 0
        self.is_turning = False  # NEW: Turn phase after backing up
        self.turn_timer = 0      # NEW: Timer for turning phase
        self.turn_direction = 1  # NEW: Direction to turn (1 or -1)
        self.sweep_direction = 1
        self.sweep_timer = 0

    def get_sensor_readings(self):
        readings = {'left': 1000, 'right': 1000, 'front': 1000}
        if self.mapper.proximity_sensors:
            front = [3, 4]
            left = [5, 6]
            right = [1, 2]
            readings['front'] = min(self.mapper.proximity_sensors[i].getValue() for i in front)
            readings['left'] = min(self.mapper.proximity_sensors[i].getValue() for i in left)
            readings['right'] = min(self.mapper.proximity_sensors[i].getValue() for i in right)
        return readings

    def navigate_to_frontier(self):
        if not self.mapper.path_to_frontier:
            return MAX_SPEED * 0.7, MAX_SPEED * 0.7
        next_point = self.mapper.path_to_frontier[0]
        dx = next_point[0] - self.mapper.position[0]
        dy = next_point[1] - self.mapper.position[1]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 5:
            self.mapper.path_to_frontier.pop(0)
            return MAX_SPEED * 0.7, MAX_SPEED * 0.7
        desired_angle = math.atan2(dy, dx)
        angle_diff = ((desired_angle - self.mapper.orientation + math.pi) % (2 * math.pi)) - math.pi
        Kp = 1.5
        turn_speed = Kp * angle_diff
        max_turn = MAX_SPEED * 0.5
        turn_speed = max(-max_turn, min(max_turn, turn_speed))
        base_speed = MAX_SPEED * 0.7
        left_speed = base_speed - turn_speed
        right_speed = base_speed + turn_speed
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > MAX_SPEED:
            scale = MAX_SPEED / max_speed
            left_speed *= scale
            right_speed *= scale
        return left_speed, right_speed

    def decide_movement_with_obstacle_avoidance(self):
        readings = self.get_sensor_readings()
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED

        # Initialize obstacle_positions list if not already present
        if not hasattr(self.mapper, 'obstacle_positions'):
            self.mapper.obstacle_positions = []

        # Stuck detection
        if readings['front'] < OBSTACLE_THRESHOLD or readings['left'] < OBSTACLE_THRESHOLD or readings['right'] < OBSTACLE_THRESHOLD:
            self.stuck_counter += 1
            # If stuck, mark the current GPS position on the occupancy map
            if self.stuck_counter > STUCK_THRESHOLD and not self.is_backing_up and not self.is_turning:
                x_grid = int(self.mapper.position[0])
                y_grid = int(self.mapper.position[1])
                x_grid = max(0, min(MAP_SIZE - 1, x_grid))
                y_grid = max(0, min(MAP_SIZE - 1, y_grid))
                # Draw a circle for stuck positions
                cv2.circle(self.mapper.visited_map, (x_grid, y_grid), 8, [64, 64, 64], -1)
        else:
            self.stuck_counter = 0

        # Trigger enhanced stuck recovery sequence
        if self.stuck_counter > STUCK_THRESHOLD and not self.is_backing_up and not self.is_turning:
            self.is_backing_up = True
            self.backup_timer = BACKUP_TIME
            self.mapper.path_to_frontier = []  # Clear any existing path
            

        # ENHANCED STUCK RECOVERY - Phase 1: Backup
        if self.is_backing_up:
            # Back up with slight randomization to avoid getting stuck in same spot
            left_speed = -MAX_SPEED * 0.5
            right_speed = -MAX_SPEED * 0.5 * (1 + random.uniform(-0.1, 0.1))
            self.backup_timer -= 1
            
            if self.backup_timer <= 0:
                # End backup phase, start turning phase
                self.is_backing_up = False
                self.is_turning = True
                self.turn_timer = TURN_TIME
                self.turn_direction = random.choice([1, -1])  # Randomly choose turn direction
                
            
            return left_speed, right_speed

        # ENHANCED STUCK RECOVERY - Phase 2: Turn 90 degrees
        if self.is_turning:
            # Turn in place to rotate approximately 90 degrees
            turn_speed = MAX_SPEED * 0.6  # Moderate turning speed
            left_speed = turn_speed * self.turn_direction
            right_speed = -turn_speed * self.turn_direction
            self.turn_timer -= 1
            
            if self.turn_timer <= 0:
                # End turning phase, reset all stuck recovery states
                self.is_turning = False
                self.stuck_counter = 0
                self.sweep_direction = random.choice([1, -1])  # Randomize future sweep direction
                
            
            return left_speed, right_speed

        # Obstacle detection and reporting for slaves
        if not self.mapper.is_master and readings['front'] < OBSTACLE_THRESHOLD:
            # Calculate obstacle position based on robot's position and orientation
            x_grid = int(self.mapper.position[0] + 15 * math.cos(self.mapper.orientation))
            y_grid = int(self.mapper.position[1] - 15 * math.sin(self.mapper.orientation))
            x_grid = max(0, min(MAP_SIZE - 1, x_grid))
            y_grid = max(0, min(MAP_SIZE - 1, y_grid))
            # Append obstacle position to shared list
            self.mapper.obstacle_positions.append((x_grid, y_grid))

        # Standard obstacle avoidance (when not in stuck recovery)
        if readings['front'] < OBSTACLE_THRESHOLD:
            turn_direction = random.choice([1, -1])
            left_speed, right_speed = MAX_SPEED * turn_direction, -MAX_SPEED * turn_direction
            self.mapper.path_to_frontier = []
        elif readings['left'] < OBSTACLE_THRESHOLD:
            left_speed, right_speed = MAX_SPEED, MAX_SPEED * 0.1
            self.mapper.path_to_frontier = []
        elif readings['right'] < OBSTACLE_THRESHOLD:
            left_speed, right_speed = MAX_SPEED * 0.1, MAX_SPEED
            self.mapper.path_to_frontier = []

        # Navigation: Master uses SLAM, slaves use coverage heuristic
        if self.mapper.is_master:
            if not (readings['front'] < OBSTACLE_THRESHOLD or
                    readings['left'] < OBSTACLE_THRESHOLD or
                    readings['right'] < OBSTACLE_THRESHOLD):
                left_speed, right_speed = self.navigate_to_frontier()
        else:
            if not (readings['front'] < OBSTACLE_THRESHOLD or
                    readings['left'] < OBSTACLE_THRESHOLD or
                    readings['right'] < OBSTACLE_THRESHOLD):
                self.sweep_timer += 1
                if self.sweep_timer > 50:
                    self.sweep_direction *= -1
                    self.sweep_timer = 0
                left_speed = MAX_SPEED * (1 + 0.2 * self.sweep_direction)
                right_speed = MAX_SPEED * (1 - 0.2 * self.sweep_direction)
                
                # Target point navigation for slaves
                if self.mapper.target_point:
                    dx = self.mapper.target_point[0] - self.mapper.position[0]
                    dy = self.mapper.target_point[1] - self.mapper.position[1]
                    desired_angle = math.atan2(dy, dx)
                    angle_diff = ((desired_angle - self.mapper.orientation + math.pi) % (2 * math.pi)) - math.pi
                    if abs(angle_diff) > 0.1:
                        turn_speed = 1.5 * angle_diff
                        left_speed -= turn_speed
                        right_speed += turn_speed
                        max_speed = max(abs(left_speed), abs(right_speed))
                        if max_speed > MAX_SPEED:
                            scale = MAX_SPEED / max_speed
                            left_speed *= scale
                            right_speed *= scale

        return left_speed, right_speed

