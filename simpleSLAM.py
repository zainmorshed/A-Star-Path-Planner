import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

class SimpleSLAM:
    def __init__(self, grid_size=200, resolution=5):
        """
        grid_size: size of map in cells
        resolution: cm per cell
        """
        self.grid_size = grid_size
        self.resolution = resolution  # cm per grid cell
        
        # Occupancy grid: 0 = unknown, 0.5 = free, 1 = occupied
        self.grid = np.ones((grid_size, grid_size)) * 0.5
        
        # Robot pose (x, y, theta) - start at center
        self.x = grid_size // 2
        self.y = grid_size // 2
        self.theta = 0  # heading in degrees
        
        # For visualization
        self.trajectory = [(self.x, self.y)]
        
    def update_pose(self, dx, dy, dtheta):
        """Update robot pose (you'll manually provide this based on driving)"""
        self.x += dx
        self.y += dy
        self.theta += dtheta
        self.theta = self.theta % 360
        self.trajectory.append((self.x, self.y))
        
    def update_map(self, distance_cm):
        """Update occupancy grid based on sensor reading"""
        if distance_cm >= 400:  # No obstacle detected
            return
            
        # Convert distance to grid cells
        distance_cells = distance_cm / self.resolution
        
        # Calculate obstacle position in grid
        theta_rad = np.radians(self.theta)
        obstacle_x = int(self.x + distance_cells * np.cos(theta_rad))
        obstacle_y = int(self.y + distance_cells * np.sin(theta_rad))
        
        # Mark obstacle as occupied (if within bounds)
        if 0 <= obstacle_x < self.grid_size and 0 <= obstacle_y < self.grid_size:
            self.grid[obstacle_y, obstacle_x] = 1.0
            
        # Ray tracing: mark cells along the ray as free
        num_steps = int(distance_cells)
        for i in range(1, num_steps):
            ray_x = int(self.x + i * np.cos(theta_rad))
            ray_y = int(self.y + i * np.sin(theta_rad))
            
            if 0 <= ray_x < self.grid_size and 0 <= ray_y < self.grid_size:
                # Mark as free (but don't overwrite obstacles)
                if self.grid[ray_y, ray_x] < 0.8:
                    self.grid[ray_y, ray_x] = 0.0
    
    def get_map_visualization(self):
        """Return map for visualization"""
        return self.grid


class SLAMVisualizer:
    def __init__(self, slam, serial_port, baud_rate=115200):
        self.slam = slam
        self.serial_port = serial_port
        self.ser = None
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.im = self.ax.imshow(slam.grid, cmap='gray_r', vmin=0, vmax=1)
        self.robot_dot, = self.ax.plot([], [], 'ro', markersize=10, label='Robot')
        self.traj_line, = self.ax.plot([], [], 'r-', linewidth=1, alpha=0.5, label='Trajectory')
        
        self.ax.set_title('SLAM Map (Manual Control)')
        self.ax.legend()
        plt.colorbar(self.im, ax=self.ax, label='Occupancy (0=free, 1=occupied)')
        
        # Manual control instructions
        self.ax.text(0.02, 0.98, 'Manual Controls:\nW/S: Forward/Back\nA/D: Left/Right', 
                    transform=self.ax.transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Connect keyboard
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
    def connect_serial(self):
        """Connect to Arduino"""
        try:
            self.ser = serial.Serial(self.serial_port, 115200, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            print(f"Connected to {self.serial_port}")
            return True
        except Exception as e:
            print(f"Error connecting: {e}")
            return False
    
    def on_key(self, event):
        """Handle manual control keypresses"""
        move_speed = 2  # cells per keypress
        turn_speed = 15  # degrees per keypress
        
        if event.key == 'w':  # Forward
            dx = move_speed * np.cos(np.radians(self.slam.theta))
            dy = move_speed * np.sin(np.radians(self.slam.theta))
            self.slam.update_pose(dx, dy, 0)
            print("Moving forward")
            
        elif event.key == 's':  # Backward
            dx = -move_speed * np.cos(np.radians(self.slam.theta))
            dy = -move_speed * np.sin(np.radians(self.slam.theta))
            self.slam.update_pose(dx, dy, 0)
            print("Moving backward")
            
        elif event.key == 'a':  # Turn left
            self.slam.update_pose(0, 0, turn_speed)
            print(f"Turning left, heading: {self.slam.theta:.1f}°")
            
        elif event.key == 'd':  # Turn right
            self.slam.update_pose(0, 0, -turn_speed)
            print(f"Turning right, heading: {self.slam.theta:.1f}°")
    
    def update_plot(self, frame):
        """Update visualization"""
        # Read sensor data from Arduino
        if self.ser and self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                
                if line.startswith("DIST"):
                    parts = line.split(',')
                    distance = float(parts[1])
                    
                    # Update map with sensor reading
                    self.slam.update_map(distance)
                    
                    print(f"Distance: {distance:.1f}cm, Pose: ({self.slam.x:.1f}, {self.slam.y:.1f}, {self.slam.theta:.1f}°)")
                    
            except Exception as e:
                print(f"Error reading serial: {e}")
        
        # Update visualization
        self.im.set_array(self.slam.grid)
        
        # Update robot position
        self.robot_dot.set_data([self.slam.x], [self.slam.y])
        
        # Update trajectory
        if len(self.slam.trajectory) > 1:
            traj = np.array(self.slam.trajectory)
            self.traj_line.set_data(traj[:, 0], traj[:, 1])
        
        return self.im, self.robot_dot, self.traj_line
    
    def run(self):
        """Start SLAM visualization"""
        if self.connect_serial():
            ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=True)
            plt.show()
        else:
            print("Failed to connect to Arduino. Check port and try again.")


# Main execution
if __name__ == "__main__":
    # Configuration
    SERIAL_PORT = '/dev/cu.usbserial-14230'  # Change to your Arduino port (COM3, COM4 on Windows, /dev/ttyUSB0 on Linux)
    
    # Create SLAM system
    slam = SimpleSLAM(grid_size=200, resolution=5)  # 200x200 grid, 5cm per cell = 10m x 10m map
    
    # Create and run visualizer
    viz = SLAMVisualizer(slam, SERIAL_PORT)
    
    print("Starting SLAM system...")
    print("Use W/A/S/D keys to indicate when you move the car:")
    print("  W = Forward")
    print("  S = Backward")
    print("  A = Turn Left")
    print("  D = Turn Right")
    print("\nDrive the car with your phone app while pressing keys to update position!")
    
    viz.run()