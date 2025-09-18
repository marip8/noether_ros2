#include <noether_ros2/conversions.h>
#include <noether_ros2/srv/plan_tool_path.hpp>

#include <fstream>
#include <noether_tpp/serialization.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

class ProcessMeshClientNode : public rclcpp::Node
{
public:
  ProcessMeshClientNode() : Node("process_mesh_client_node")
  {
    declare_parameter<std::string>("config_file", "");
    declare_parameter<std::string>("mesh_file", "");

    rclcpp::Client<noether_ros2::srv::PlanToolPath>::SharedPtr client_ =
        this->create_client<noether_ros2::srv::PlanToolPath>("plan_tool_path");

    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<noether_ros2::srv::PlanToolPath::Request>();

    // Load the config file into a YAML string for the request
    std::string config_file;
    get_parameter("config_file", config_file);
    request->config = YAML::Dump(YAML::LoadFile(config_file));

    // Load the mesh into the request
    std::string mesh_file;
    get_parameter("mesh_file", mesh_file);
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFile(mesh_file, mesh) == -1)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Couldn't read file: " << mesh_file);
      return;
    }
    request->mesh = noether_ros2::toMsg(mesh);

    // Call the service
    auto future = client_->async_send_request(request);
    future.wait();
    noether_ros2::srv::PlanToolPath::Response::SharedPtr response = future.get();

    // Handle the response
    if (!response->success)
    {
      RCLCPP_ERROR_STREAM(get_logger(), response->message);
      return;
    }

    // Save the tool paths to a file
    YAML::Node node;
    for (std::size_t i = 0; i < response->tool_paths.size(); ++i)
    {
      const noether_ros2::msg::ToolPaths& tool_paths_msg = response->tool_paths[i];
      node.push_back(noether_ros2::fromMsg(tool_paths_msg));
    }

    std::ofstream file("/tmp/tpp.yaml");
    file << node;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProcessMeshClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
