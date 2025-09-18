#include <noether_ros2/conversions.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace noether_ros2
{
pcl::PolygonMesh fromMsg(const pcl_msgs::msg::PolygonMesh& msg)
{
  pcl::PolygonMesh mesh;

  // Convert the header
  pcl_conversions::toPCL(msg.header, mesh.header);

  // Convert the point cloud data
  pcl_conversions::toPCL(msg.cloud, mesh.cloud);

  // Convert the polygons
  mesh.polygons.reserve(msg.polygons.size());
  for (const auto& polygon_msg : msg.polygons)
  {
    pcl::Vertices vertices;
    vertices.vertices.reserve(polygon_msg.vertices.size());

    for (auto v : polygon_msg.vertices)
      vertices.vertices.push_back(static_cast<int>(v));

    mesh.polygons.push_back(vertices);
  }

  return mesh;
}

pcl_msgs::msg::PolygonMesh toMsg(const pcl::PolygonMesh& mesh)
{
  pcl_msgs::msg::PolygonMesh msg;

  // Convert the header
  pcl_conversions::fromPCL(mesh.header, msg.header);

  // Convert the point cloud data
  pcl_conversions::fromPCL(mesh.cloud, msg.cloud);

  // Convert the polygons
  msg.polygons.reserve(mesh.polygons.size());
  for (const pcl::Vertices& polygon : mesh.polygons)
  {
    pcl_msgs::msg::Vertices polygon_msg;
    polygon_msg.vertices.reserve(polygon.vertices.size());

    for (auto v : polygon.vertices)
      polygon_msg.vertices.push_back(static_cast<unsigned>(v));

    msg.polygons.push_back(polygon_msg);
  }

  return msg;
}

geometry_msgs::msg::PoseArray toMsg(const noether::ToolPathSegment& segment)
{
  geometry_msgs::msg::PoseArray segment_msg;
  segment_msg.poses.reserve(segment.size());

  std::transform(segment.begin(),
                 segment.end(),
                 std::back_inserter(segment_msg.poses),
                 [](const Eigen::Isometry3d& pose) { return tf2::toMsg(pose); });

  return segment_msg;
}

noether::ToolPathSegment fromMsg(const geometry_msgs::msg::PoseArray& segment_msg)
{
  noether::ToolPathSegment segment;
  segment.reserve(segment_msg.poses.size());

  std::transform(segment_msg.poses.begin(),
                 segment_msg.poses.end(),
                 std::back_inserter(segment),
                 [](const geometry_msgs::msg::Pose& msg) {
                   Eigen::Isometry3d pose;
                   tf2::fromMsg(msg, pose);
                   return pose;
                 });

  return segment;
}

noether_ros2::msg::ToolPath toMsg(const noether::ToolPath& tool_path)
{
  noether_ros2::msg::ToolPath tool_path_msg;
  tool_path_msg.segments.reserve(tool_path.size());

  std::transform(tool_path.begin(),
                 tool_path.end(),
                 std::back_inserter(tool_path_msg.segments),
                 [](const noether::ToolPathSegment& segment) { return toMsg(segment); });

  return tool_path_msg;
}

noether::ToolPath fromMsg(const noether_ros2::msg::ToolPath& tool_path_msg)
{
  noether::ToolPath tool_path;
  tool_path.reserve(tool_path_msg.segments.size());

  std::transform(tool_path_msg.segments.begin(),
                 tool_path_msg.segments.end(),
                 std::back_inserter(tool_path),
                 [](const geometry_msgs::msg::PoseArray& msg) { return fromMsg(msg); });

  return tool_path;
}

noether_ros2::msg::ToolPaths toMsg(const noether::ToolPaths& tool_paths)
{
  noether_ros2::msg::ToolPaths tool_paths_msg;
  tool_paths_msg.tool_paths.reserve(tool_paths.size());

  std::transform(tool_paths.begin(),
                 tool_paths.end(),
                 std::back_inserter(tool_paths_msg.tool_paths),
                 [](const noether::ToolPath& tool_path) { return toMsg(tool_path); });

  return tool_paths_msg;
}

noether::ToolPaths fromMsg(const noether_ros2::msg::ToolPaths& tool_paths_msg)
{
  noether::ToolPaths tool_paths;
  tool_paths.reserve(tool_paths_msg.tool_paths.size());

  std::transform(tool_paths_msg.tool_paths.begin(),
                 tool_paths_msg.tool_paths.end(),
                 std::back_inserter(tool_paths),
                 [](const noether_ros2::msg::ToolPath& msg) { return fromMsg(msg); });

  return tool_paths;
}

geometry_msgs::msg::PoseArray toMsg(const std::vector<noether::ToolPaths>& tool_paths_list)
{
  geometry_msgs::msg::PoseArray msg;

  for (const noether::ToolPaths& tool_paths : tool_paths_list)
    for (const noether::ToolPath& tool_path : tool_paths)
      for (const noether::ToolPathSegment& segment : tool_path)
        for (const Eigen::Isometry3d& pose : segment)
          msg.poses.push_back(tf2::toMsg(pose));

  return msg;
}

}  // namespace noether_ros2
