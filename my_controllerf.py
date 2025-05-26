from controller import Robot
from roomba_mapper import RoombaMapper
import math

def main():
    robot = Robot()
    robot_name = robot.getName()
    is_master = "master" in robot_name.lower()
    
    if is_master:
        for i in range(8):
            led = robot.getDevice(f'led{i}')
            if led:
                led.set(1)
    
    mapper = RoombaMapper(robot, is_master=is_master)
    
    while robot.step(128) != -1:
        mapper.run_step()

if __name__ == "__main__":
    main()