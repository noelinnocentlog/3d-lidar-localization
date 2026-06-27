#!/usr/bin/env python3

import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix


class MapSaverNode(Node):
    def __init__(self):
        super().__init__('save_map')
        self.declare_parameter('map_dir', '')
        self.declare_parameter('map_name', 'map')

    def save(self):
        map_dir_param = self.get_parameter('map_dir').value
        if map_dir_param:
            map_dir = Path(map_dir_param).expanduser().resolve()
        else:
            # go up from install/<pkg> to the workspace root
            workspace_root = Path(get_package_prefix('opensource_2d_mapping')).parent.parent
            map_dir = workspace_root / 'maps'
        base_name = self.get_parameter('map_name').value
        map_dir.mkdir(parents=True, exist_ok=True)

        map_path = map_dir / _next_name(map_dir, base_name)
        self.get_logger().info(f'Saving map to: {map_path}')

        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(map_path)],
                check=True,
            )
            self.get_logger().info(f'Map saved: {map_path}')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to save map: {e}')


def _next_name(map_dir: Path, base_name: str) -> str:
    i = 1
    while True:
        name = f'{base_name}_{i:03d}'
        if not (map_dir / f'{name}.yaml').exists() and not (map_dir / f'{name}.pgm').exists():
            return name
        i += 1


def main():
    rclpy.init()
    node = MapSaverNode()
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
