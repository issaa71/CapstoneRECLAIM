#!/usr/bin/env python3
"""
Load planning scene with environment collision objects (bins, mounts, base)
and a ground plane. Run after move_group is up.

Usage:
    ros2 run reclaim_arm_moveit_config load_planning_scene
    # or directly:
    python3 load_planning_scene.py
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header, ColorRGBA
import struct
import os
import time


def load_stl_binary(filepath):
    """Load a binary STL file and return a shape_msgs/Mesh."""
    mesh = Mesh()

    with open(filepath, 'rb') as f:
        # Skip 80-byte header
        f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]

        vertex_map = {}
        vertices = []

        for _ in range(num_triangles):
            # Skip normal (12 bytes)
            f.read(12)

            triangle = MeshTriangle()
            indices = []

            for _ in range(3):
                vx, vy, vz = struct.unpack('<3f', f.read(12))
                # Scale mm to meters
                vx *= 0.001
                vy *= 0.001
                vz *= 0.001

                key = (round(vx, 6), round(vy, 6), round(vz, 6))
                if key not in vertex_map:
                    p = Point()
                    p.x = vx
                    p.y = vy
                    p.z = vz
                    vertex_map[key] = len(vertices)
                    vertices.append(p)
                indices.append(vertex_map[key])

            triangle.vertex_indices = indices
            mesh.triangles.append(triangle)

            # Skip attribute byte count (2 bytes)
            f.read(2)

    mesh.vertices = vertices
    return mesh


class PlanningSceneLoader(Node):
    def __init__(self):
        super().__init__('planning_scene_loader')

        self.scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10
        )
        self.collision_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10
        )

        # Wait for subscribers
        time.sleep(2.0)
        self.get_logger().info('Loading planning scene...')

        self.load_environment()
        self.load_ground()
        self.publish_colors()

        self.get_logger().info('Planning scene loaded successfully!')

    def load_environment(self):
        """Load the environment STL (bins, mounts, wooden base)."""
        # Find the STL file
        meshes_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'meshes'
        )
        stl_path = os.path.join(meshes_dir, 'environment.stl')

        if not os.path.exists(stl_path):
            # Try installed path
            import ament_index_python
            pkg_dir = ament_index_python.get_package_share_directory(
                'reclaim_arm_moveit_config'
            )
            stl_path = os.path.join(pkg_dir, 'meshes', 'environment.stl')

        if not os.path.exists(stl_path):
            self.get_logger().error(f'STL not found: {stl_path}')
            return

        self.get_logger().info(f'Loading STL from: {stl_path}')
        mesh = load_stl_binary(stl_path)
        self.get_logger().info(
            f'Loaded mesh: {len(mesh.vertices)} vertices, '
            f'{len(mesh.triangles)} triangles'
        )

        # Create collision object
        co = CollisionObject()
        co.header.frame_id = 'base_link'
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = 'environment'
        co.operation = CollisionObject.ADD

        # STL origin is at J1 — no offset needed
        mesh_pose = Pose()
        mesh_pose.position.x = -0.005  # 5mm back to clear search pose collision
        mesh_pose.position.y = 0.0
        mesh_pose.position.z = 0.0
        mesh_pose.orientation.w = 1.0

        co.meshes.append(mesh)
        co.mesh_poses.append(mesh_pose)

        self.collision_pub.publish(co)
        self.get_logger().info('Published environment collision object')

    def load_ground(self):
        """Add a ground plane as a flat box."""
        co = CollisionObject()
        co.header.frame_id = 'base_link'
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = 'ground'
        co.operation = CollisionObject.ADD

        # Box primitive: 2m x 2m x 1cm
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 2.0, 0.01]

        # Ground is 210mm below J1
        ground_pose = Pose()
        ground_pose.position.x = 0.0
        ground_pose.position.y = 0.0
        ground_pose.position.z = -0.215  # 210mm below J1 + 5mm margin
        ground_pose.orientation.w = 1.0

        co.primitives.append(box)
        co.primitive_poses.append(ground_pose)

        self.collision_pub.publish(co)
        self.get_logger().info('Published ground collision object')

    def publish_colors(self):
        """Set solid opaque colors for collision objects."""
        ps = PlanningScene()
        ps.is_diff = True

        # Environment: steel blue, slightly transparent
        env_color = ObjectColor()
        env_color.id = 'environment'
        env_color.color = ColorRGBA()
        env_color.color.r = 0.4
        env_color.color.g = 0.6
        env_color.color.b = 0.8
        env_color.color.a = 0.7

        # Ground: flat dark slate
        ground_color = ObjectColor()
        ground_color.id = 'ground'
        ground_color.color = ColorRGBA()
        ground_color.color.r = 0.2
        ground_color.color.g = 0.2
        ground_color.color.b = 0.25
        ground_color.color.a = 0.8

        ps.object_colors.append(env_color)
        ps.object_colors.append(ground_color)

        self.scene_pub.publish(ps)
        self.get_logger().info('Published object colors (solid opaque)')


def main():
    rclpy.init()
    node = PlanningSceneLoader()

    # Publish a few times to ensure delivery
    for _ in range(3):
        time.sleep(1.0)
        node.load_environment()
        node.load_ground()
        node.publish_colors()

    node.get_logger().info('Done. Shutting down.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
