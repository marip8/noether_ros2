#pragma once

#include <noether_ros/msg/tool_paths.hpp>
#include <noether_tpp/core/types.h>

namespace noether_ros
{
geometry_msgs::msg::PoseArray toMsg(const noether::ToolPathSegment& segment, const std::string frame = "");
noether::ToolPathSegment toMsg(const geometry_msgs::msg::PoseArray& segment_msg);

noether_ros::msg::ToolPath toMsg(const noether::ToolPath& tool_path, const std::string frame = "");
noether::ToolPath fromMsg(const noether_ros::msg::ToolPath& tool_path_msg);

noether_ros::msg::ToolPaths toMsg(const noether::ToolPaths& tool_paths, const std::string frame = "");
noether::ToolPaths fromMsg(const noether_ros::msg::ToolPaths& tool_paths_msg);

std::vector<noether_ros::msg::ToolPaths> toMsg(const std::vector<noether::ToolPaths>& tool_paths_list,
                                               const std::string& frame = "");
std::vector<noether::ToolPaths> fromMsg(const std::vector<noether_ros::msg::ToolPaths>& tool_paths_msg);

geometry_msgs::msg::PoseArray flattenToMsg(const noether::ToolPath& tool_paths, const std::string frame = "");
geometry_msgs::msg::PoseArray flattenToMsg(const noether::ToolPaths& tool_paths, const std::string frame = "");
geometry_msgs::msg::PoseArray flattenToMsg(const std::vector<noether::ToolPaths>& tool_paths,
                                           const std::string frame = "");

}  // namespace noether_ros
