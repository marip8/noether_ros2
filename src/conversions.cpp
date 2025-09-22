#include <noether_ros/conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace noether_ros
{
geometry_msgs::msg::PoseArray toMsg(const noether::ToolPathSegment& segment, const std::string frame)
{
  geometry_msgs::msg::PoseArray segment_msg;
  segment_msg.header.frame_id = frame;
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

noether_ros::msg::ToolPath toMsg(const noether::ToolPath& tool_path, const std::string frame)
{
  noether_ros::msg::ToolPath tool_path_msg;
  tool_path_msg.segments.reserve(tool_path.size());

  std::transform(tool_path.begin(),
                 tool_path.end(),
                 std::back_inserter(tool_path_msg.segments),
                 [&frame](const noether::ToolPathSegment& segment) { return toMsg(segment, frame); });

  return tool_path_msg;
}

noether::ToolPath fromMsg(const noether_ros::msg::ToolPath& tool_path_msg)
{
  noether::ToolPath tool_path;
  tool_path.reserve(tool_path_msg.segments.size());

  std::transform(tool_path_msg.segments.begin(),
                 tool_path_msg.segments.end(),
                 std::back_inserter(tool_path),
                 [](const geometry_msgs::msg::PoseArray& msg) { return fromMsg(msg); });

  return tool_path;
}

noether_ros::msg::ToolPaths toMsg(const noether::ToolPaths& tool_paths, const std::string frame)
{
  noether_ros::msg::ToolPaths tool_paths_msg;
  tool_paths_msg.tool_paths.reserve(tool_paths.size());

  std::transform(tool_paths.begin(),
                 tool_paths.end(),
                 std::back_inserter(tool_paths_msg.tool_paths),
                 [&frame](const noether::ToolPath& tool_path) { return toMsg(tool_path, frame); });

  return tool_paths_msg;
}

noether::ToolPaths fromMsg(const noether_ros::msg::ToolPaths& tool_paths_msg)
{
  noether::ToolPaths tool_paths;
  tool_paths.reserve(tool_paths_msg.tool_paths.size());

  std::transform(tool_paths_msg.tool_paths.begin(),
                 tool_paths_msg.tool_paths.end(),
                 std::back_inserter(tool_paths),
                 [](const noether_ros::msg::ToolPath& msg) { return fromMsg(msg); });

  return tool_paths;
}

geometry_msgs::msg::PoseArray toMsg(const std::vector<noether::ToolPaths>& tool_paths_list, const std::string frame)
{
  geometry_msgs::msg::PoseArray msg;
  msg.header.frame_id = frame;

  for (const noether::ToolPaths& tool_paths : tool_paths_list)
    for (const noether::ToolPath& tool_path : tool_paths)
      for (const noether::ToolPathSegment& segment : tool_path)
        for (const Eigen::Isometry3d& pose : segment)
          msg.poses.push_back(tf2::toMsg(pose));

  return msg;
}

}  // namespace noether_ros
