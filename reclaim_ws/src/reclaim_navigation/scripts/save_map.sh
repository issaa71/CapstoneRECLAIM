#!/bin/bash
# Save the current SLAM Toolbox map to disk
#
# Usage:
#   ./save_map.sh                    # saves to default location
#   ./save_map.sh my_custom_map      # saves with custom name
#
# This calls map_saver_cli which saves two files:
#   <name>.yaml  — map metadata (resolution, origin, etc.)
#   <name>.pgm   — occupancy grid image
#
# Requires SLAM Toolbox to be running and publishing /map

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPS_DIR="${SCRIPT_DIR}/../maps"
MAP_NAME="${1:-venue_map}"
MAP_PATH="${MAPS_DIR}/${MAP_NAME}"

echo "Saving map to: ${MAP_PATH}"
echo "  Output files: ${MAP_PATH}.yaml, ${MAP_PATH}.pgm"

# Ensure the maps directory exists
mkdir -p "${MAPS_DIR}"

# Source ROS2 setup
if [ -f ~/reclaim_ws/install/setup.bash ]; then
    source ~/reclaim_ws/install/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Save the map
ros2 run nav2_map_server map_saver_cli -f "${MAP_PATH}" --ros-args -p save_map_timeout:=5000

echo ""
echo "Map saved successfully!"
echo "To use this map with nav_only.launch.py:"
echo "  ros2 launch reclaim_navigation nav_only.launch.py map:=${MAP_PATH}.yaml"
