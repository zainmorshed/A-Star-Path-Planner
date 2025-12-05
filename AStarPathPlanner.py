import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import heapq


class AStarPathPlanner:
    def __init__(self, grid_size=10):
#grid size]

        self.grid_size = grid_size
        self.grid = np.zeros((grid_size, grid_size))  # 0 =free, 1 = obstacle
        
   #state
        self.robot_pos = [0, 0]     
        self.robot_heading = 270      #start - rboto faces south
        self.goal_pos = [grid_size-1, grid_size-1]  #goal

    def set_obstacle(self, row, col):
        
#mark cell as obstacle
        if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
            self.grid[row, col] = 1

    def set_goal(self, row, col):
        self.goal_pos = [row, col]      #goal position

    def heuristic(self, pos1, pos2):
#huerisitc - using the manhattan distance
        return abs(pos1[0]-pos2[0])+abs(pos1[1]-pos2[1])

    def get_neighbors(self, pos):
#get the valid neighboring cells
        row, col = pos
        neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  #up, down, left,right
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if 0 <= new_row < self.grid_size and 0 <= new_col < self.grid_size:
                if self.grid[new_row, new_col] == 0:
                    neighbors.append([new_row, new_col])
        return neighbors



    def astar(self, start, goal):
#returns path as a list
        counter = 0
        start_tuple = tuple(start)
        goal_tuple = tuple(goal)
        h = self.heuristic(start, goal)
        pq = [(h, counter, start_tuple, [start_tuple])]
        counter += 1
        visited = set()

        while pq:
            f_score, _, current, path = heapq.heappop(pq)
            if current in visited:
                continue
            visited.add(current)
            if current == goal_tuple:
                return [list(p) for p in path]

            current_list = list(current)
            for neighbor in self.get_neighbors(current_list):
                neighbor_tuple = tuple(neighbor)
                if neighbor_tuple not in visited:
                    g_score = len(path)
                    h_score = self.heuristic(neighbor, goal)
                    f_score = g_score + h_score
                    new_path = path + [neighbor_tuple]
                    heapq.heappush(pq, (f_score, counter, neighbor_tuple, new_path))
                    counter+=1
        return None             #if there is not path found 
    
    

    def plan_path(self):
        """Plan a path from current position to goal"""
        path = self.astar(self.robot_pos, self.goal_pos)
        if path is None:
            print("No path found!")
            return None
        print(f"Path found with {len(path)} steps")
        return path




# controller
class RobotController:
    def __init__(self, serial_port, baud_rate=9600):
        self.ser = None
        self.serial_port = serial_port
        self.baud_rate = baud_rate


    def connect(self):
#connect to the arudiuno
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=2)
            time.sleep(2)
            response = self.ser.readline().decode('utf-8').strip()
            if response == "READY":
                print(f"Connected to Arduino on {self.serial_port}")
                return True
        except Exception as e:
            print(f"Error connecting: {e}")
            return False
        return False

    def send_command(self, command):        #sending command to the arduino 
        if not self.ser:
            print("Not connected!")
            return False
        try:

#buffer for SCAN
            if command == "SCAN":
                try:
                    self.ser.reset_input_buffer()
                except:
                    pass

            self.ser.write(f"{command}\n".encode())

            start = time.time()
            timeout = 2.0
            while True:
                if (time.time() - start) > timeout:
                    return False
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                if command == "SCAN":
                    if line.startswith("DIST:"):
                        try:
                            distance = float(line.split(":", 1)[1])
                            return distance
                        except:
                            continue
                    else:
                        continue
                else:
                    if line == "DONE":
                        return True
                    else:
                        continue
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
        return False

    def scan_obstacle(self):
        return self.send_command("SCAN")

    def move_forward(self):
        return self.send_command("FORWARD")

    def turn_left(self):
        return self.send_command("LEFT")

    def turn_right(self):
        return self.send_command("RIGHT")

    def stop(self):
        return self.send_command("STOP")

# path executiuon

class PathExecutor:
    def __init__(self, planner, controller):
        self.planner = planner
        self.controller = controller

    def get_direction_to_neighbor(self, current, next_pos):
        dr = next_pos[0] - current[0]
        dc = next_pos[1] - current[1]
        if dr == -1:
            return 90
        elif dr == 1:
            return 270
        elif dc == -1:
            return 180
        elif dc == 1:
            return 0
        return None

    def turn_to_heading(self, target_heading):
        current = self.planner.robot_heading
        diff = (target_heading - current) % 360
        if diff == 0:
            return
        elif diff == 90 or diff == -270:
            print("Turning left...")
            self.controller.turn_left()
            self.planner.robot_heading = target_heading
        elif diff == 270 or diff == -90:
            print("Turning right...")
            self.controller.turn_right()
            self.planner.robot_heading = target_heading
        elif diff == 180:
            print("Turning around...")
            self.controller.turn_left()
            self.controller.turn_left()
            self.planner.robot_heading = target_heading

    def execute_path(self, path, obstacle_threshold=50):
        if not path or len(path) < 2:
            print("No path to execute")
            return False

        i = 0
        while i < len(path)-1:
            current = path[i]
            next_pos = path[i + 1]
            print(f"\nStep {i+1}/{len(path)-1}: Moving from {current} to {next_pos}")

            target_heading = self.get_direction_to_neighbor(current, next_pos)
            self.turn_to_heading(target_heading)

            print("Scanning for obstacles...")
            time.sleep(0.3)
            distance = self.controller.scan_obstacle()
            if distance is False:
                print("Scan timed out, assuming clear")
                distance = 400
            print(f"Distance ahead: {distance:.1f} cm")

            if distance == 0.0 or distance > 300:
                distance = 400

            if distance < obstacle_threshold:
                print("Obstacle detected! Marking grid and re-planning...")
                self.planner.set_obstacle(next_pos[0], next_pos[1])
                return False     #signal the main loop to re-plan

            print("Moving forward...")
            self.controller.move_forward()
            self.planner.robot_pos =next_pos
            i+=1
            time.sleep(0.3)

        return True

# visualzition - path plan

class Visualizer:
    def __init__(self, planner):
        self.planner = planner
        self.fig, self.ax = plt.subplots(figsize=(8,8))

    def draw_grid(self, path=None):
        self.ax.clear()
        for i in range(self.planner.grid_size):
            for j in range(self.planner.grid_size):
                color = 'white'
                if self.planner.grid[i, j] == 1:
                    color = 'black'
                rect = Rectangle((j, self.planner.grid_size - 1 - i), 1, 1, facecolor=color, edgecolor='gray')
                self.ax.add_patch(rect)

        if path:
            path_array = np.array(path)
            path_x = path_array[:,1] + 0.5
            path_y = self.planner.grid_size - path_array[:,0] - 0.5
            self.ax.plot(path_x, path_y, 'b-', linewidth=3, alpha=0.6, label='Planned path')

        robot_x = self.planner.robot_pos[1] +0.5
        robot_y = self.planner.grid_size - self.planner.robot_pos[0] - 0.5
        self.ax.plot(robot_x, robot_y, 'go', markersize=20, label='Robot')

        goal_x = self.planner.goal_pos[1] +0.5
        goal_y = self.planner.grid_size - self.planner.goal_pos[0] - 0.5
        self.ax.plot(goal_x, goal_y, 'r*', markersize=30, label='Goal')

        self.ax.set_xlim(0, self.planner.grid_size)
        self.ax.set_ylim(0, self.planner.grid_size)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.legend()
        self.ax.set_title('A* Path Planning')
        plt.pause(0.1)



if __name__ == "__main__":
    SERIAL_PORT = '/dev/tty.usbserial-14110'  #update  PORT!!!
    GRID_SIZE = 6
    OBSTACLE_THRESHOLD = 50

    print("=== A* Path Planning Robot ===\n")

    planner = AStarPathPlanner(grid_size=GRID_SIZE)

#goal
    goal_row = int(input(f"Enter goal row (0-{GRID_SIZE-1}): "))
    goal_col = int(input(f"Enter goal column (0-{GRID_SIZE-1}): "))
    planner.set_goal(goal_row, goal_col)

#connect to the arduino
    print("\nConnecting to robot...")
    controller = RobotController(SERIAL_PORT)
    if not controller.connect():
        print("Failed to connect. Exiting.")
        exit()

    executor = PathExecutor(planner, controller)
    viz = Visualizer(planner)

#keep planning until goal is reached

    while planner.robot_pos != planner.goal_pos:
        print(f"\nCurrent position: {planner.robot_pos}, planning path to goal {planner.goal_pos}...")
        path = planner.plan_path()
        if path is None:
            print("No path found! Exiting.")
            break

        viz.draw_grid(path)
        plt.show(block=False)
        input("Press Enter to execute path (or Ctrl+C to quit)...")

        success = executor.execute_path(path, OBSTACLE_THRESHOLD)
        if success:
            print("\n Successfully reached goal!")
            break
        else:
            print("Obstacle detected, re-planning...")
            viz.draw_grid()
            plt.show(block=False)
            controller.stop()

    print("\nProgram complete!")
    plt.show()
