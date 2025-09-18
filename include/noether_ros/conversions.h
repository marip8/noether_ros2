#pragma once

#include <noether_ros/msg/tool_paths.hpp>
#include <noether_tpp/core/types.h>

namespace noether_ros
{
geometry_msgs::msg::PoseArray toMsg(const noether::ToolPathSegment& segment);
noether::ToolPathSegment toMsg(const geometry_msgs::msg::PoseArray& segment_msg);

noether_ros::msg::ToolPath toMsg(const noether::ToolPath& tool_path);
noether::ToolPath fromMsg(const noether_ros::msg::ToolPath& tool_path_msg);

noether_ros::msg::ToolPaths toMsg(const noether::ToolPaths& tool_paths);
noether::ToolPaths fromMsg(const noether_ros::msg::ToolPaths& tool_paths_msg);

geometry_msgs::msg::PoseArray toMsg(const std::vector<noether::ToolPaths>& tool_paths);

}  // namespace noether_ros
