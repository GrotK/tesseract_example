#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <trajopt_common/collision_types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/basic_cartesian_example.h>

#include <tesseract_common/timer.h>

#include <tesseract_collision/core/types.h>

#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

#include <tesseract_state_solver/state_solver.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/plotting.h>

#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <tesseract_task_composer/planning/planning_task_composer_problem.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_msgs/msg/environment_state.hpp> 
#include <tesseract_geometry/impl/octree.h>
#include <tesseract_geometry/impl/octree_utils.h>
#include <rclcpp/rclcpp.hpp>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_geometry/impl/sphere.h>

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
using tesseract_common::ManipulatorInfo;

static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";




class ExampleNode : public rclcpp::Node
{
public:
    ExampleNode() : Node("example_traj"), env_(std::make_shared<tesseract_environment::Environment>())
    {
        subscription_ = this->create_subscription<tesseract_msgs::msg::EnvironmentState>(
            "/tesseract_ros_examples/tesseract_published_environment", 10,
            std::bind(&ExampleNode::environment_callback, this, std::placeholders::_1));
    }

    void environment_callback(const tesseract_msgs::msg::EnvironmentState::SharedPtr msg)
    {
        if (!env_->isInitialized())
        {
           // RCLCPP_ERROR(this->get_logger(), "env not initialized");
        }

        // Update environment state with received state
        //env_->setState(msg->joint_state->name, msg->joint_state);
    }

    void run_example()
    {
  bool ifopt_ = true;  // or false

  std::vector<std::string> joint_names;
  joint_names.emplace_back("joint_a1");
  joint_names.emplace_back("joint_a2");
  joint_names.emplace_back("joint_a3");
  joint_names.emplace_back("joint_a4");
  joint_names.emplace_back("joint_a5");
  joint_names.emplace_back("joint_a6");
  joint_names.emplace_back("joint_a7");

  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  auto env_ = std::make_shared<Environment>();
  auto plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>("base_link", "tesseract_ros_examples");
  if (plotter_ != nullptr)
    plotter_->waitForConnection();

  //std::string urdf_path = "/opt/tesseract/install/tesseract_support/share/tesseract_support/urdf/abb_irb2400.urdf";
  //std::string srdf_path = "/opt/tesseract/install/tesseract_support/share/tesseract_support/urdf/abb_irb2400.srdf";

  std::string urdf_path = "/opt/tesseract/install/tesseract_support/share/tesseract_support/urdf/lbr_iiwa_14_r820.urdf";
  std::string srdf_path = "/opt/tesseract/install/tesseract_support/share/tesseract_support/urdf/lbr_iiwa_14_r820.srdf";

  // Load URDF and SRDF strings
  std::ifstream urdf_file(urdf_path);
  std::ifstream srdf_file(srdf_path);
  std::stringstream urdf_ss, srdf_ss;
  urdf_ss << urdf_file.rdbuf();
  srdf_ss << srdf_file.rdbuf();
  std::string urdf_string = urdf_ss.str();
  std::string srdf_string = srdf_ss.str();

  env_->init(urdf_string, srdf_string, locator);

  // Add sphere to environment
  Link link_sphere("sphere_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.5, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Sphere>(0.25);
  link_sphere.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_sphere.collision.push_back(collision);

  Joint joint_sphere("joint_sphere_attached");
  joint_sphere.parent_link_name = "base_link";
  joint_sphere.child_link_name = link_sphere.getName();
  joint_sphere.type = JointType::FIXED;

  

  Command::Ptr cmd = std::make_shared<tesseract_environment::AddLinkCommand>(link_sphere, joint_sphere);
    env_->applyCommand(cmd);

  // Create Task Composer Plugin Factory
  const std::string share_dir(TESSERACT_TASK_COMPOSER_DIR);
  tesseract_common::fs::path config_path(share_dir + "/config/task_composer_plugins.yaml");
  TaskComposerPluginFactory factory(config_path);
  // Create Program
  CompositeInstruction program(
      "cartesian_program", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool0"));

  // Start Joint Position for the program
  auto joint_values = env_->getStateSolver()->getState().joints; 
  Eigen::VectorXd position(joint_values.size());
int i = 0;
for (const auto& pair : joint_values) {
    position[i++] = pair.second;
}
if (joint_names.size() != position.size()) {
    throw std::runtime_error("Joint names and position have different sizes!");

}
RCLCPP_INFO(this->get_logger(), "Joint Names:");
for (const auto& name : joint_names) {
    RCLCPP_INFO(this->get_logger(), "  %s", name.c_str());
}

// Wypisz wartości stawów
RCLCPP_INFO(this->get_logger(), "Joint Values:");
for (const auto& pair : joint_values) {
    RCLCPP_INFO(this->get_logger(), "  %s: %f", pair.first.c_str(), pair.second);
}

  StateWaypointPoly wp0{ StateWaypoint(joint_names, position) };
  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
  start_instruction.setDescription("Start Instruction");

  // Create cartesian waypoint
  CartesianWaypointPoly wp1{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.37,0.40,  0.71)  *
                                               Eigen::Quaterniond(0, 0,  0, 1.0)) };

  CartesianWaypointPoly wp2{ CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d( 0.37,-0.40, 0.71)  * //x y z //x w strone kostki z wysokosc y w lewo
                                               Eigen::Quaterniond(0, 0,  0, 1.0)) };


  // Plan freespace from start
  MoveInstruction plan_f0(wp1, MoveInstructionType::FREESPACE, "freespace_profile");
  plan_f0.setDescription("from_start_plan");

  // Plan linear move
  MoveInstruction plan_c0(wp2, MoveInstructionType::FREESPACE, "RASTER");

  // Plan freespace to end
  MoveInstruction plan_f1(wp0, MoveInstructionType::FREESPACE, "freespace_profile");
  plan_f1.setDescription("to_end_plan");

  // Add Instructions to program
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f0);
  program.appendMoveInstruction(plan_c0);
  program.appendMoveInstruction(plan_f1);

  // Print Diagnostics
  program.print("Program: ");

  // Create Executor
  auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");

  // Create profile dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  if (ifopt_)
  {
    RCLCPP_INFO(this->get_logger(), "ifopt");

    auto composite_profile = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
    composite_profile->collision_cost_config->type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    composite_profile->collision_constraint_config->type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
    composite_profile->smooth_velocities = true;
    composite_profile->smooth_accelerations = false;
    composite_profile->smooth_jerks = false;
    composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
    profiles->addProfile<TrajOptIfoptCompositeProfile>(
        TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "cartesian_program", composite_profile);

    auto plan_profile = std::make_shared<TrajOptIfoptDefaultPlanProfile>();
    plan_profile->cartesian_coeff = Eigen::VectorXd::Ones(6);
    plan_profile->joint_coeff = Eigen::VectorXd::Ones(7);
    profiles->addProfile<TrajOptIfoptPlanProfile>(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "RASTER", plan_profile);
    profiles->addProfile<TrajOptIfoptPlanProfile>(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "freespace_profile", plan_profile);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "not ifopt");

    auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    composite_profile->collision_cost_config.enabled = true;
    composite_profile->collision_constraint_config.enabled = true;
    composite_profile->smooth_velocities = true;
    composite_profile->smooth_accelerations = false;
    composite_profile->smooth_jerks = false;
    composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
    profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "cartesian_program", composite_profile);

    auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
    plan_profile->cartesian_cost_config.enabled = false;
    plan_profile->cartesian_constraint_config.enabled = true;
    plan_profile->cartesian_constraint_config.coeff = Eigen::VectorXd::Ones(6);
    plan_profile->joint_cost_config.enabled = false;
    plan_profile->joint_constraint_config.enabled = true;
    plan_profile->joint_constraint_config.coeff = Eigen::VectorXd::Ones(7);
    profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "RASTER", plan_profile);
    profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "freespace_profile", plan_profile);
  }

  // Create task
  const std::string task_name = (ifopt_) ? "TrajOptIfoptPipeline" : "TrajOptPipeline";
  TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
  const std::string output_key = task->getOutputKeys().front();

  // Create Task Composer Problem
  auto problem = std::make_unique<PlanningTaskComposerProblem>(env_, profiles);
  problem->input = program;

  if (plotter_ != nullptr && plotter_->isConnected())
    plotter_->waitForInput("Hit Enter to solve for trajectory.");

  // Solve task
  tesseract_common::Timer stopwatch;
  stopwatch.start();
  TaskComposerFuture::UPtr future = executor->run(*task, std::move(problem));
  future->wait();

  stopwatch.stop();

  // Plot Process Trajectory
  if (plotter_ != nullptr && plotter_->isConnected())
  {
    plotter_->waitForInput();
    auto ci = future->context->data_storage->getData(output_key).as<CompositeInstruction>();
    tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
    tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
    auto state_solver = env_->getStateSolver();
    plotter_->plotMarker(ToolpathMarker(toolpath));
    plotter_->plotTrajectory(trajectory, *state_solver);
    RCLCPP_INFO(this->get_logger(), "wyplotowalo");

  }
  const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";
  auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(shared_from_this(),env_, EXAMPLE_MONITOR_NAMESPACE);
  monitor->startPublishingEnvironment();

}
    

private:
    rclcpp::Subscription<tesseract_msgs::msg::EnvironmentState>::SharedPtr subscription_;
    std::shared_ptr<tesseract_environment::Environment> env_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExampleNode>();
    // Spin in a separate thread to allow the environment to be updated
    std::thread([node]() { rclcpp::spin(node); }).detach();


    // Run the example after some delay to ensure the environment is received
    std::this_thread::sleep_for(std::chrono::seconds(1));
    node->run_example();

    rclcpp::shutdown();
    return 0;
}