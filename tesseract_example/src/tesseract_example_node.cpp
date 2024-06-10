#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tesseract_common/resource_locator.h>
#include <tesseract_environment/environment.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_monitoring/environment_monitor.h>

#include <tesseract_rosutils/plotting.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/utils.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_environment/environment.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <descartes_light/edge_evaluators/compound_edge_evaluator.h>
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/fwd.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_motion_planners/robot_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>

using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_srdf;
using namespace tesseract_rosutils;
using namespace tesseract_planning;
using namespace tesseract_common;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_visualization;

bool validate_urdf_srdf(const std::string& urdf_path, const std::string& srdf_path)
{
  std::ifstream urdf_file(urdf_path);
  std::ifstream srdf_file(srdf_path);

  if (!urdf_file || !srdf_file)
  {
    std::cerr << "Failed to open URDF or SRDF file!" << std::endl;
    return false;
  }

  std::stringstream urdf_ss, srdf_ss;
  urdf_ss << urdf_file.rdbuf();
  srdf_ss << srdf_file.rdbuf();

  std::string urdf_string = urdf_ss.str();
  std::string srdf_string = srdf_ss.str();

  if (urdf_string.empty() || srdf_string.empty())
  {
    std::cerr << "URDF or SRDF string is empty!" << std::endl;
    return false;
  }

  std::cout << "URDF and SRDF files are successfully read and are not empty." << std::endl;
  return true;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tesseract_ros2_example");
  // Locate URDF and SRDF files
  //std::string urdf_path = "/opt/tesseract/install/tesseract_support/share/tesseract_support/urdf/abb_irb2400.urdf";
  //std::string srdf_path = "/opt/tesseract/install/tesseract_support/share/tesseract_support/urdf/abb_irb2400.srdf";
    std::string urdf_path = "/opt/tesseract/install/tesseract_support/share/tesseract_support/urdf/lbr_iiwa_14_r820.urdf";
  std::string srdf_path = "/opt/tesseract/install/tesseract_support/share/tesseract_support/urdf/lbr_iiwa_14_r820.srdf";
  // Load URDF and SRDF strings
  if (!validate_urdf_srdf(urdf_path, srdf_path))
  {
    std::cerr << "Validation failed!" << std::endl;
    return -1;
  }

  std::cout << "Validation succeeded!" << std::endl;

  std::ifstream urdf_file(urdf_path);
  std::ifstream srdf_file(srdf_path);
  std::stringstream urdf_ss, srdf_ss;
  urdf_ss << urdf_file.rdbuf();
  srdf_ss << srdf_file.rdbuf();
  std::string urdf_string = urdf_ss.str();
  std::string srdf_string = srdf_ss.str();

  // Create Tesseract Environment
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  auto tesseract = std::make_shared<Environment>();
  
  // Initialize Environment
  tesseract->init(urdf_string, srdf_string, locator);
///////////////
  // Create a publisher for the joint states
  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // Create a static transform broadcaster
  auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  // Create a plotting instance
  // Note: ROSPlotting does not need node or environment as constructor arguments
  auto plotter = std::make_shared<tesseract_rosutils::ROSPlotting>("world", "tesseract");

  auto publish_static_transforms = [static_broadcaster]() {
  geometry_msgs::msg::TransformStamped transformStamped;
  
  // Konfiguracja przykładowej statycznej transformacji
  transformStamped.header.stamp = rclcpp::Clock().now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = 0.0;
  transformStamped.transform.rotation.w = 1.0;

  static_broadcaster->sendTransform(transformStamped);
};
  publish_static_transforms();


const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";
  auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(node, tesseract, EXAMPLE_MONITOR_NAMESPACE);
  monitor->startPublishingEnvironment();
  // Set up a timer to publish the joint states at a regular interval










  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100), 
    [tesseract, joint_state_pub, static_broadcaster]() {
      // Publish the current state of the robot
      auto current_state = tesseract->getStateSolver()->getState(); // Use getStateSolver()->getState() instead
      auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
      joint_state_msg->header.stamp = rclcpp::Clock().now();
      const auto& joint_names = tesseract->getJointNames();
      const auto& joint_values = current_state.joints;

      joint_state_msg->name = joint_names;

      for (const auto& joint_name : joint_names) {
        if (joint_values.find(joint_name) != joint_values.end()) { // Check if the joint name exists in the map
          joint_state_msg->position.push_back(joint_values.at(joint_name));
          //RCLCPP_WARN(rclcpp::get_logger("tesseract_ros2_example"), "Joint name %s found in joint values! %f", joint_name.c_str(), joint_values.find(joint_name));

        } else {
          //RCLCPP_WARN(rclcpp::get_logger("tesseract_ros2_example"), "Joint name %s not found in joint values!", joint_name.c_str());
          joint_state_msg->position.push_back(0.0); // Set default value if not found
        }
      }
      joint_state_pub->publish(*joint_state_msg);
      // Publish static transforms (if any)
      // Here, we should add the static transforms if necessary
    });


  

  



  // Spin ROS node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
