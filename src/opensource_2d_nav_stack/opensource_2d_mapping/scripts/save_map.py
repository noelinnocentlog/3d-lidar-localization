#!/usr/bin/env python3

import subprocess
from pathlib import Path


def main():
    workspace = Path.home() / "ros2_masterproject"
    map_dir = workspace / "src" / "opensource_2d_nav_stack" / "opensource_2d_mapping" / "maps"

    map_dir.mkdir(parents=True, exist_ok=True)

    map_name = "office_map"
    map_path = map_dir / map_name

    print(f"Saving map to: {map_path}")

    try:
        subprocess.run(
            [
                "ros2",
                "run",
                "nav2_map_server",
                "map_saver_cli",
                "-f",
                str(map_path)
            ],
            check=True
        )

        print("✅ Map saved successfully!")

    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to save map: {e}")


if __name__ == "__main__":
    main()