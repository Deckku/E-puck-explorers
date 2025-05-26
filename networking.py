import socket
import pickle
import struct
import threading
import time
from queue import Queue
import math
import random

# Constants
SERVER_PORT = 5555

class Networking:
    def __init__(self, mapper, is_master):
        self.mapper = mapper
        if is_master:
            self.server_thread = threading.Thread(target=self.run_server)
            self.server_thread.daemon = True
            self.server_thread.start()
            self.mapper.frontier_queue = Queue()
            self.mapper.assigned_targets = {}
        else:
            self.client_socket = None
            self.connect_to_master()

    def connect_to_master(self):
        for attempt in range(5):
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect(('127.0.0.1', SERVER_PORT))
                send_thread = threading.Thread(target=self.send_updates_to_master)
                send_thread.daemon = True
                send_thread.start()
                receive_thread = threading.Thread(target=self.receive_updates_from_master)
                receive_thread.daemon = True
                receive_thread.start()
                return
            except Exception:
                time.sleep(2)

    def send_gps_update(self):
        if self.mapper.is_master or not self.client_socket:
            return
        try:
            data = {
                'robot_id': self.mapper.robot_id,
                'position': self.mapper.position.copy(),
                'orientation': self.mapper.orientation
            }
            serialized_data = pickle.dumps(data)
            size = struct.pack('!I', len(serialized_data))
            self.client_socket.sendall(size + serialized_data)
        except Exception:
            self.client_socket = None
            self.connect_to_master()

    def send_updates_to_master(self):
        while True:
            if self.client_socket:
                self.send_gps_update()
            time.sleep(0.5)

    def receive_updates_from_master(self):
        while True:
            if self.client_socket:
                try:
                    size_data = self.client_socket.recv(4)
                    if not size_data:
                        break
                    size = struct.unpack('!I', size_data)[0]
                    received_data = b''
                    while len(received_data) < size:
                        chunk = self.client_socket.recv(min(4096, size - len(received_data)))
                        if not chunk:
                            break
                        received_data += chunk
                    data = pickle.loads(received_data)
                    self.mapper.other_robots_data = data.get('robots_data', {})
                    if 'target' in data and data['robot_id'] == self.mapper.robot_id:
                        self.mapper.target_point = data['target']
                except Exception:
                    self.client_socket = None
                    self.connect_to_master()
                    break
            time.sleep(0.5)

    def run_server(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server_socket.bind(('0.0.0.0', SERVER_PORT))
            server_socket.listen(5)
            client_sockets = []
            while True:
                server_socket.settimeout(1.0)
                try:
                    client_sock, addr = server_socket.accept()
                    client_sockets.append(client_sock)
                    client_thread = threading.Thread(target=self.handle_client, args=(client_sock,))
                    client_thread.daemon = True
                    client_thread.start()
                except socket.timeout:
                    self.mapper.update_frontiers()
                    self.assign_targets_to_robots(client_sockets)
                    all_robots_data = self.mapper.other_robots_data.copy()
                    all_robots_data[self.mapper.robot_id] = {
                        'position': self.mapper.position.copy(),
                        'orientation': self.mapper.orientation
                    }
                    serialized_data = pickle.dumps({'robots_data': all_robots_data})
                    size = struct.pack('!I', len(serialized_data))
                    for client_sock in client_sockets[:]:
                        try:
                            client_sock.sendall(size + serialized_data)
                        except:
                            client_sockets.remove(client_sock)
                            client_sock.close()
        except Exception:
            pass
        finally:
            server_socket.close()

    def handle_client(self, client_socket):
        try:
            while True:
                size_data = client_socket.recv(4)
                if not size_data:
                    break
                size = struct.unpack('!I', size_data)[0]
                received_data = b''
                while len(received_data) < size:
                    chunk = client_socket.recv(min(4096, size - len(received_data)))
                    if not chunk:
                        break
                    received_data += chunk
                robot_data = pickle.loads(received_data)
                robot_id = robot_data['robot_id']
                self.mapper.other_robots_data[robot_id] = {
                    'position': robot_data['position'],
                    'orientation': robot_data['orientation']
                }
                self.mapper.mapping.update_map_from_gps(robot_id, robot_data['position'], robot_data['orientation'])
        except Exception:
            pass
        finally:
            client_socket.close()

    def assign_targets_to_robots(self, client_sockets):
        robots = {rid: data for rid, data in self.mapper.other_robots_data.items()}
        robots[self.mapper.robot_id] = {
            'position': self.mapper.position,
            'orientation': self.mapper.orientation
        }
        for rid in list(self.mapper.assigned_targets.keys()):
            if rid not in robots:
                del self.mapper.assigned_targets[rid]
                continue
            pos = robots[rid]['position']
            target = self.mapper.assigned_targets[rid]
            dist = math.hypot(pos[0] - target[0], pos[1] - target[1])
            if dist < 10:
                del self.mapper.assigned_targets[rid]
        available_robots = [rid for rid in robots if rid not in self.mapper.assigned_targets]
        for rid in available_robots:
            if not self.mapper.frontier_queue.empty():
                target = self.mapper.frontier_queue.get()
            else:
                target = [self.mapper.position[0] + random.randint(-50, 50),
                          self.mapper.position[1] + random.randint(-50, 50)]
            self.mapper.assigned_targets[rid] = target
            if rid == self.mapper.robot_id:
                self.mapper.target_point = target
            else:
                data = {
                    'robot_id': rid,
                    'target': target,
                    'robots_data': robots
                }
                serialized_data = pickle.dumps(data)
                size = struct.pack('!I', len(serialized_data))
                for client_sock in client_sockets[:]:
                    try:
                        client_sock.sendall(size + serialized_data)
                    except:
                        client_sockets.remove(client_sock)
                        client_sock.close()