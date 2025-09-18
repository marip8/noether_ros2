#include <noether_ros2/srv/plan_tool_path.hpp>
#include <noether_ros2/conversions.h>

#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <noether_tpp/plugin_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

class ToolPathPlanningServer : public rclcpp::Node
{
public:
  ToolPathPlanningServer() : Node("tool_path_planning_server")
  {
    server_ = this->create_service<noether_ros2::srv::PlanToolPath>(
        "plan_tool_path",
        std::bind(&ToolPathPlanningServer::callback, this, std::placeholders::_1, std::placeholders::_2));
  }

protected:
  void callback(const std::shared_ptr<noether_ros2::srv::PlanToolPath::Request> request,
                std::shared_ptr<noether_ros2::srv::PlanToolPath::Response> response)
  {
    try
    {
      // Convert input string to YAML node
      YAML::Node config = YAML::Load(request->config);

      // Create and run the tool path planning pipeline
      noether::Factory factory;
      noether::ToolPathPlannerPipeline pipeline(factory, config);
      std::vector<noether::ToolPaths> tool_paths = pipeline.plan(noether_ros2::fromMsg(request->mesh));

      // Fill in the response
      response->success = true;
      response->tool_paths.reserve(tool_paths.size());
      std::transform(tool_paths.begin(),
                     tool_paths.end(),
                     std::back_inserter(response->tool_paths),
                     [](const noether::ToolPaths& tp) { return noether_ros2::toMsg(tp); });
    }
    catch (const std::exception& ex)
    {
      response->success = false;
      response->message = ex.what();
    }
  }

  rclcpp::Service<noether_ros2::srv::PlanToolPath>::SharedPtr server_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ToolPathPlanningServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
