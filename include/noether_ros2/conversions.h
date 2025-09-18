#pragma once

#include <noether_ros2/msg/tool_paths.hpp>
#include <noether_tpp/core/types.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <pcl/PolygonMesh.h>

namespace noether_ros2
{
pcl_msgs::msg::PolygonMesh toMsg(const pcl::PolygonMesh& mesh);
pcl::PolygonMesh fromMsg(const pcl_msgs::msg::PolygonMesh& msg);

geometry_msgs::msg::PoseArray toMsg(const noether::ToolPathSegment& segment);
noether::ToolPathSegment toMsg(const geometry_msgs::msg::PoseArray& segment_msg);

noether_ros2::msg::ToolPath toMsg(const noether::ToolPath& tool_path);
noether::ToolPath fromMsg(const noether_ros2::msg::ToolPath& tool_path_msg);

noether_ros2::msg::ToolPaths toMsg(const noether::ToolPaths& tool_paths);
noether::ToolPaths fromMsg(const noether_ros2::msg::ToolPaths& tool_paths_msg);

geometry_msgs::msg::PoseArray toMsg(const std::vector<noether::ToolPaths>& tool_paths);

}  // namespace noether_ros2
