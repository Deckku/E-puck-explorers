from controller import Robot
import numpy as np
import cv2
from navigation import Navigation
from mapping import Mapping
from networking import Networking
import math
import time
from ultralytics import YOLO

# Constants
TIME_STEP = 128
MAP_SIZE = 1000
MAP_RESOLUTION = 0.05
FOV_ANGLE = math.radians(60)
FOV_RANGE = 50
CAMERA_FOV = math.radians(60)
MAX_OBJECT_DISTANCE = 1.0

class RoombaMapper:
    def __init__(self, robot, is_master=False):
        self.robot = robot
        self.left_motor = robot.getDevice('left wheel motor')
        self.right_motor = robot.getDevice('right wheel motor')
        self.gps = robot.getDevice('gps')
        self.compass = robot.getDevice('compass')
        self.robot_id = robot.getName()
        self.is_master = is_master
        
        # Initialize motors
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Enable sensors
        self.gps.enable(TIME_STEP)
        self.compass.enable(TIME_STEP)
        
        # Initialize proximity sensors
        self.proximity_sensors = [robot.getDevice(f'ps{i}') for i in range(8)]
        for sensor in self.proximity_sensors:
            if sensor:
                sensor.enable(TIME_STEP)
        
        # Initialize camera
        self.camera = robot.getDevice('HDcamera')
        if self.camera:
            self.camera.enable(TIME_STEP)
            self.camera_width = self.camera.getWidth()
            self.camera_height = self.camera.getHeight()
        else:
            print(f"Robot {self.robot_id}: Camera not found")
        
        # Initialize YOLOv8
        self.yolo_model = self._load_yolo_model()
        
        # Initialize map and state
        self.visited_map = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        self.position = [MAP_SIZE // 2, MAP_SIZE // 2]
        self.orientation = 0
        self.step_count = 0
        self.other_robots_data = {}
        self.target_point = [MAP_SIZE // 2 + 50, MAP_SIZE // 2] if is_master else None
        self.obstacle_positions = []
        
        # SLAM variables
        self.frontier_clusters = []
        self.last_frontier_update = 0
        self.target_frontier = None
        self.path_to_frontier = []
        
        # Helper modules
        self.navigation = Navigation(self)
        self.mapping = Mapping(self)
        self.networking = Networking(self, is_master)

    def _load_yolo_model(self):
        try:
            return YOLO('yolov8n.pt')
        except Exception as e:
            print(f"Robot {self.robot_id}: Failed to load YOLOv8: {e}")
            return None

    def _world_to_grid(self, x_world, y_world):
        scale = MAP_SIZE / 2
        x_grid = int((x_world * scale) + scale)
        y_grid = int((-y_world * scale) + scale)
        return max(0, min(MAP_SIZE - 1, x_grid)), max(0, min(MAP_SIZE - 1, y_grid))

    def calculate_fov_triangle(self, position, orientation):
        x, y = position
        range_grid = FOV_RANGE
        angle_left = orientation - FOV_ANGLE / 2
        angle_right = orientation + FOV_ANGLE / 2
        point_a = (int(x), int(y))
        point_b = (int(x + range_grid * math.cos(angle_left)), int(y - range_grid * math.sin(angle_left)))
        point_c = (int(x + range_grid * math.cos(angle_right)), int(y - range_grid * math.sin(angle_right)))
        return np.array([point_a, point_b, point_c], np.int32)

    def mark_fov_on_map(self, triangle):
        triangle = np.clip(triangle, 0, MAP_SIZE - 1)
        cv2.fillPoly(self.visited_map, [triangle], color=(0, 0, 255))

    def process_camera_image(self):
        if not self.camera or not self.yolo_model:
            return
        
        # Get and convert image
        image = self.camera.getImage()
        if not image:
            return
        image_array = np.frombuffer(image, dtype=np.uint8).reshape((self.camera_height, self.camera_width, 4))
        image_bgr = cv2.cvtColor(image_array, cv2.COLOR_BGRA2BGR)
        
        # Run YOLOv8
        results = self.yolo_model(image_bgr, verbose=False)
        
        # Process detections
        for result in results:
            for box in result.boxes:
                if box.conf.item() < 0.5:
                    continue
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                class_name = self.yolo_model.names[int(box.cls.item())]
                confidence = box.conf.item()
                
                # Calculate object position
                box_center_x = (x_min + x_max) / 2
                angle_offset = ((box_center_x - self.camera_width / 2) / self.camera_width) * CAMERA_FOV
                object_angle = self.orientation + angle_offset
                x_world = (self.position[0] / (MAP_SIZE / 2)) * (MAP_SIZE / 2) + MAX_OBJECT_DISTANCE * math.cos(object_angle)
                y_world = (self.position[1] / (MAP_SIZE / 2)) * (MAP_SIZE / 2) - MAX_OBJECT_DISTANCE * math.sin(object_angle)
                
                if class_name == "potted plant" or class_name == "truck" or class_name == "dining table" or class_name == "bench":
                    print(f"Robot {self.robot_id}: Detected fellow e-puck warrior (confidence: {confidence:.2f}, at world ({x_world:.2f}, {y_world:.2f}))")
                else:
                    print(f"Robot {self.robot_id}: Detected {class_name} (confidence: {confidence:.2f}, at world ({x_world:.2f}, {y_world:.2f}))")
                
                # Map to grid and mark
                x_grid, y_grid = self._world_to_grid(x_world, y_world)
                cv2.circle(self.visited_map, (x_grid, y_grid), 5, (255, 0, 255), -1)
                
                # Report to master if slave
                if not self.is_master:
                    self.obstacle_positions.append((x_grid, y_grid))

    def run_step(self):
        self.step_count += 1
        
        # Update position and process camera
        self.mapping.update_position()
        self.process_camera_image()
        
        # Mark robot's FOV
        my_fov_triangle = self.calculate_fov_triangle(self.position, self.orientation)
        self.mark_fov_on_map(my_fov_triangle)
        
        # Master tasks: update map and mark other robots' FOV
        if self.is_master:
            self.mapping.update_map_from_proximity()
            self.mapping.update_map_from_slave_obstacles()
            for robot_id, data in self.other_robots_data.items():
                if 'position' in data and 'orientation' in data:
                    fov_triangle = self.calculate_fov_triangle(data['position'], data['orientation'])
                    self.mark_fov_on_map(fov_triangle)
                    x_grid, y_grid = map(int, data['position'])
                    x_grid, y_grid = max(0, min(MAP_SIZE - 1, x_grid)), max(0, min(MAP_SIZE - 1, y_grid))
                    current_pos = (x_grid, y_grid)
                    if 'prev_position' in data and data['prev_position']:
                        prev_x, prev_y = data['prev_position']
                        if 0 <= prev_x < MAP_SIZE and 0 <= prev_y < MAP_SIZE:
                            cv2.line(self.visited_map, (prev_x, prev_y), current_pos, [255, 255, 255], thickness=10)
                    data['prev_position'] = current_pos
        
        # Navigate and update motors
        left_speed, right_speed = self.navigation.decide_movement_with_obstacle_avoidance()
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
        
        # Update orientation
        self.orientation -= (right_speed - left_speed) / 2 * (TIME_STEP / 1000.0)
        self.orientation %= 2 * math.pi
        
        # Network update
        self.networking.send_gps_update()
        
        # Visualize map (master only)
        if self.is_master:
            map_vis = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
            map_vis[np.all(self.visited_map == [255, 255, 255], axis=2)] = [255, 255, 255]  # Paths: White
            map_vis[np.all(self.visited_map == [128, 128, 128], axis=2)] = [0, 255, 0]  # Obstacles: Green
            map_vis[np.all(self.visited_map == [64, 64, 64], axis=2)] = [0, 0, 255]  # Stuck: Red
            map_vis[np.all(self.visited_map == [0, 0, 255], axis=2)] = [255, 0, 0]  # Seen: Blue
            map_vis[np.all(self.visited_map == [255, 0, 255], axis=2)] = [128, 0, 128]  # Objects: Purple
            
            current_fov = self.calculate_fov_triangle(self.position, self.orientation)
            if self.is_triangle_in_bounds(current_fov):
                cv2.fillPoly(map_vis, [current_fov], color=(0, 255, 255))
            
            for robot_id, data in self.other_robots_data.items():
                if 'position' in data and 'orientation' in data:
                    other_fov = self.calculate_fov_triangle(data['position'], data['orientation'])
                    if self.is_triangle_in_bounds(other_fov):
                        cv2.fillPoly(map_vis, [other_fov], color=(0, 255, 255))
            
            cv2.circle(map_vis, (int(self.position[0]), int(self.position[1])), 15, (0, 255, 0), -1)
            for robot_id, data in self.other_robots_data.items():
                if 'position' in data:
                    cv2.circle(map_vis, (int(data['position'][0]), int(data['position'][1])), 15, (0, 255, 255), -1)
            if self.target_point:
                cv2.circle(map_vis, self.target_point, 10, (255, 0, 0), -1)
            
            map_vis_resized = cv2.resize(map_vis, (400, 400))
            cv2.putText(map_vis_resized, f"Robot ID: {self.robot_id}", (10, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Combined Occupancy Map", map_vis_resized)
            cv2.waitKey(1)
        
        return None, self.visited_map.copy()

    def is_triangle_in_bounds(self, triangle):
        return (0 <= min(triangle[:, 0]) < MAP_SIZE and 0 <= max(triangle[:, 0]) < MAP_SIZE and
                0 <= min(triangle[:, 1]) < MAP_SIZE and 0 <= max(triangle[:, 1]) < MAP_SIZE)

    def update_frontiers(self):
        if time.time() - self.last_frontier_update < 2.0:
            return
        self.last_frontier_update = time.time()
        
        combined_map = np.any(self.visited_map > 0, axis=2).astype(np.uint8) * 255
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(combined_map, kernel)
        frontiers = dilated - combined_map
        
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(frontiers.astype(np.uint8))
        self.frontier_clusters = []
        min_cluster_size = 20
        
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] >= min_cluster_size:
                y, x = np.where(labels == i)
                centroid = (int(np.mean(x)), int(np.mean(y)))
                self.frontier_clusters.append({
                    'points': list(zip(x, y)),
                    'centroid': centroid,
                    'size': stats[i, cv2.CC_STAT_AREA]
                })
        self.frontier_clusters.sort(key=lambda x: -x['size'])

    def select_target_frontier(self):
        if not self.frontier_clusters:
            return None
        current_pos = (int(self.position[0]), int(self.position[1]))
        best_score = -float('inf')
        best_frontier = None
        
        for frontier in self.frontier_clusters:
            dx, dy = frontier['centroid'][0] - current_pos[0], frontier['centroid'][1] - current_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            min_robot_dist = float('inf')
            for robot_id, data in self.other_robots_data.items():
                if robot_id != self.robot_id and 'position' in data:
                    fdx = frontier['centroid'][0] - int(data['position'][0])
                    fdy = frontier['centroid'][1] - int(data['position'][1])
                    min_robot_dist = min(min_robot_dist, math.sqrt(fdx*fdx + fdy*fdy))
            score = frontier['size'] / 100.0 + 50.0 / max(1.0, distance) + min(50.0, min_robot_dist) / 50.0
            if score > best_score:
                best_score = score
                best_frontier = frontier
        return best_frontier

    def plan_path_to_frontier(self, frontier):
        current_pos = (int(self.position[0]), int(self.position[1]))
        target_pos = frontier['centroid']
        path = [current_pos]
        x, y = current_pos
        max_steps = 100
        
        while (x, y) != target_pos and len(path) < max_steps:
            dx, dy = target_pos[0] - x, target_pos[1] - y
            x += 1 if dx > 0 else -1 if dx < 0 else 0
            y += 1 if dy > 0 else -1 if dy < 0 else 0
            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE and not np.all(self.visited_map[y, x] == [128, 128, 128]):
                path.append((x, y))
            else:
                break
        return path