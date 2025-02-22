import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
import os

class MapReader(Node):
    def __init__(self):
        super().__init__('map_reader')
        self.model_path = os.path.expanduser('~/.gazebo/models/engineering_quad/')
        self.load_map_from_sdf()

    def load_map_from_sdf(self):
        # Load model.config to find the SDF file path
        config_path = os.path.join(self.model_path, 'model.config')
        sdf_file_path = self.get_sdf_path(config_path)

        # Parse model.sdf file
        if sdf_file_path:
            map_data = self.parse_sdf(sdf_file_path)
            if map_data is not None:
                self.plot_occupancy_grid(map_data)

    def get_sdf_path(self, config_path):
        try:
            tree = ET.parse(config_path)
            root = tree.getroot()
            sdf_filename = root.find('model').find('sdf').text
            sdf_file_path = os.path.join(self.model_path, sdf_filename)
            return sdf_file_path if os.path.exists(sdf_file_path) else None
        except Exception as e:
            self.get_logger().error(f"Error reading model.config: {e}")
            return None

    def parse_sdf(self, sdf_file_path):
        try:
            tree = ET.parse(sdf_file_path)
            root = tree.getroot()
            world = root.find('.//world')
            
            # Initialize an empty occupancy grid
            grid_size = (100, 100)  # Example grid size; adjust based on model scale
            occupancy_grid = np.ones(grid_size) * 255  # Initialize as free space

            # Extract obstacle positions (assuming rectangular shapes)
            for model in world.findall('model'):
                position = model.find('pose').text.split()
                x, y = float(position[0]), float(position[1])

                # Map x, y position to grid and mark as occupied (simplified)
                grid_x = int((x + 10) * 5)  # Adjust scaling as necessary
                grid_y = int((y + 10) * 5)

                # Mark a 1-cell area as occupied for this example
                if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
                    occupancy_grid[grid_y, grid_x] = 0  # Mark as occupied

            return occupancy_grid

        except Exception as e:
            self.get_logger().error(f"Error reading model.sdf: {e}")
            return None

    def plot_occupancy_grid(self, occupancy_grid):
        plt.imshow(occupancy_grid, cmap='binary', origin='lower')
        plt.colorbar()
        plt.title('Occupancy Grid from SDF')
        plt.xlabel('X (cells)')
        plt.ylabel('Y (cells)')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    map_reader = MapReader()
    rclpy.spin(map_reader)
    map_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
