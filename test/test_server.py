from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
import launch_testing
from noether_ros.srv import PlanToolPath
import os
import rclpy
from rclpy.node import Node
import unittest


def generate_test_description():
    return LaunchDescription([
            LaunchNode(
                package='noether_ros',
                executable='noether_ros_tool_path_planning_server',
                output='screen'
        ),
        launch_testing.actions.ReadyToTest()
    ])


config_str = R"""
mesh_modifiers:
  - name: NormalsFromMeshFaces
tool_path_planner:
  name: Multi
  gui_plugin_name: CrossHatchPlaneSlicer
  planners:
    - name: PlaneSlicer
      direction_generator:
        name: PrincipalAxis
        rotation_offset: 1.571
      origin_generator:
        name: AABBCenter
      line_spacing: 0.1
      point_spacing: 0.025
      min_hole_size: 0.100
      search_radius: 0.100
      min_segment_size: 0.100
      bidirectional: true
    - name: PlaneSlicer
      direction_generator:
        name: PCARotated
        direction_generator:
          name: PrincipalAxis
          rotation_offset: 1.571
        rotation_offset: 1.571
      origin_generator:
        name: AABBCenter
      line_spacing: 0.100
      point_spacing: 0.025
      min_hole_size: 0.100
      search_radius: 0.100
      min_segment_size: 0.100
      bidirectional: true
tool_path_modifiers:
  - name: SnakeOrganization
"""

# Active test
class TestToolPathPlanningClient(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        rclpy.init()
        self.node = rclpy.create_node("test_client")

    @classmethod
    def tearDownClass(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_tool_path_planning_client(self):
        # Create the client
        client = self.node.create_client(PlanToolPath, 'plan_tool_path')

        # Check that the service is available
        self.assertTrue(client.wait_for_service(timeout_sec=5.0), "Service does not exist")

        # Create the request
        request = PlanToolPath.Request()

        # Add the TPP config string to the request
        request.config = config_str

        # Add the mesh file to the request
        pkg_prefix = get_package_share_directory('noether_ros')
        request.mesh_file = os.path.join(pkg_prefix, 'test', 'dome.ply')

        # Check that the mesh file is valid
        self.assertIsNotNone(pkg_prefix)
        self.assertTrue(os.path.exists(request.mesh_file))

        # Call the service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Check the response
        response = future.result()
        self.assertIsNotNone(response, "No response received")
        self.assertTrue(response.success, response.message)
        self.assertEqual(len(response.tool_paths), 1)
        for tool_paths in response.tool_paths:
            self.assertGreaterEqual(len(tool_paths.tool_paths), 1)
            for tool_path in tool_paths.tool_paths:
                self.assertGreaterEqual(len(tool_path.segments), 1)
                for segment in tool_path.segments:
                    self.assertGreaterEqual(len(segment.poses), 1)

        self.node.destroy_client(client)
