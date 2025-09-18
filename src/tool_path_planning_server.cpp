#include <noether_ros/srv/plan_tool_path.hpp>
#include <noether_ros/conversions.h>

#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <noether_tpp/plugin_interface.h>
#include <pcl/io/vtk_lib_io.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

class ToolPathPlanningServer : public rclcpp::Node
{
public:
  ToolPathPlanningServer() : Node("tool_path_planning_server")
  {
    server_ = this->create_service<noether_ros::srv::PlanToolPath>(
        "plan_tool_path",
        std::bind(&ToolPathPlanningServer::callback, this, std::placeholders::_1, std::placeholders::_2));
  }

protected:
  void callback(const std::shared_ptr<noether_ros::srv::PlanToolPath::Request> request,
                std::shared_ptr<noether_ros::srv::PlanToolPath::Response> response)
  {
    try
    {
      // Convert input configuration string to YAML node
      YAML::Node config = YAML::Load(request->config);

      // Load the mesh file
      pcl::PolygonMesh mesh;
      if (pcl::io::loadPolygonFile(request->mesh_file, mesh) < 0)
        throw std::runtime_error("Failed to load mesh from file: " + request->mesh_file);

      // Create and run the tool path planning pipeline
      noether::Factory factory;
      noether::ToolPathPlannerPipeline pipeline(factory, config);
      std::vector<noether::ToolPaths> tool_paths = pipeline.plan(mesh);

      // Fill in the response
      response->success = true;
      response->tool_paths.reserve(tool_paths.size());
      std::transform(tool_paths.begin(),
                     tool_paths.end(),
                     std::back_inserter(response->tool_paths),
                     [](const noether::ToolPaths& tp) { return noether_ros::toMsg(tp); });
    }
    catch (const std::exception& ex)
    {
      response->success = false;
      response->message = ex.what();
    }
  }

  rclcpp::Service<noether_ros::srv::PlanToolPath>::SharedPtr server_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ToolPathPlanningServer>();
  RCLCPP_INFO_STREAM(node->get_logger(), "Started tool path planning server");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
