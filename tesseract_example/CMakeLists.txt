cmake_minimum_required(VERSION 3.5)
project(tesseract_example)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")

find_package(tesseract_collision REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tesseract_examples REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_rosutils REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_srdf REQUIRED)
find_package(tesseract_urdf REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tesseract_monitoring REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(trajopt REQUIRED)
find_package(tesseract_state_solver REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_task_composer REQUIRED)

include_directories(
  "/opt"
  ${rclcpp_INCLUDE_DIRS}
  "/opt/tesseract/install/tesseract_environment/include"
  "/opt/tesseract/install/tesseract_environment"
  "/opt/tesseract/install/tesseract_scene_graph/include/"
  "/opt/tesseract/install/tesseract_state_solver/include/"
    "/opt/tesseract/install/tesseract_srdf/include/"
    "/opt/tesseract/install/tesseract_kinematics/include/"
    "/opt/tesseract/install/tesseract_collision/include/"
    "/opt/tesseract/install/tesseract_urdf/include/"
    "/opt/tesseract_ros2/install/tesseract_rosutils/include"
  "/opt/tesseract_ros2/install/tesseract_msgs/include"
  "/opt/tesseract_planning/install/tesseract_motion_planners/include/"
  "/opt/tesseract_planning/install/tesseract_motion_planners/include/"
  "/opt/tesseract_planning/install/tesseract_command_language/include/"
  "/opt/tesseract_planning/install/descartess_light/include/"
  "/opt/tesseract_planning/install/ifopt/include/"
    "/opt/tesseract_planning/install/ros_industrial_cmake_boilerplate/include/"
  "/opt/tesseract_planning/install/ruckig/include/"
  "/opt/tesseract_planning/install/tesseract_command_language/include/"
  "/opt/tesseract_planning/install/tesseract_examples/include/"
  "/opt/tesseract_planning/install/tesseract_task_composer/include/"
  "/opt/tesseract_planning/install/tesseract_time_parametrization/include/"
  "/opt/tesseract_ros/install/tesseract_time_parametrization/include/"
  "/opt/tesseract_ros2/install/tesseract_planning_server/include/"
  "/opt/trajopt/install/trajopt_sqp/include/"
  "/opt/trajopt/install/trajopt/include/"
  "/opt/trajopt/install/trajopt_common/include/"
  "/opt/trajopt/install/trajopt_sco/include/"
  "/opt/trajopt/install/trajopt_sco/include/"
  "/opt/tesseract_planning/install/tesseract_motion_planners/include/tesseract_motion_planners/trajopt_ifopt/"
  "/opt/tesseract_planning/install/descartes_light/include/"

  ${trajopt_ifopt_INCLUDE_DIRS}
  ${tesseract_scene_graph_INCLUDE_DIRS}
  ${tesseract_common_INCLUDE_DIRS}
  ${tesseract_srdf_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${tesseract_geometry_INCLUDE_DIRS}
  
)
message(STATUS "rclcpp_INCLUDE_DIRS: ${rclcpp_INCLUDE_DIRS}")
message(STATUS "tesseract_environment_INCLUDE_DIRS: ${tesseract_environment_INCLUDE_DIRS}")
message(STATUS "tesseract_scene_graph_INCLUDE_DIRS: ${tesseract_scene_graph_INCLUDE_DIRS}")
message(STATUS "tesseract_common_INCLUDE_DIRS: ${tesseract_common_INCLUDE_DIRS}")
message(STATUS "tesseract_srdf_INCLUDE_DIRS: ${tesseract_srdf_INCLUDE_DIRS}")



add_executable(tesseract_example_node src/tesseract_example_node.cpp)
add_executable(example_traj src/example_traj.cpp)



ament_target_dependencies(tesseract_example_node tesseract_visualization trajopt rclcpp ament_index_cpp tesseract_environment tesseract_scene_graph tesseract_common tesseract_srdf tesseract_collision tesseract_command_language tesseract_environment tesseract_examples tesseract_geometry tesseract_kinematics tesseract_urdf tesseract_task_composer tesseract_state_solver tesseract_srdf tesseract_rosutils tesseract_msgs tesseract_motion_planners tesseract_monitoring trajopt_ifopt)
ament_target_dependencies(example_traj tesseract_visualization trajopt rclcpp ament_index_cpp tesseract_environment tesseract_scene_graph tesseract_common tesseract_srdf tesseract_collision tesseract_command_language tesseract_environment tesseract_examples tesseract_geometry tesseract_kinematics tesseract_urdf tesseract_task_composer tesseract_state_solver tesseract_srdf tesseract_rosutils tesseract_msgs tesseract_motion_planners tesseract_monitoring trajopt_ifopt)

target_link_libraries(example_traj
/opt/trajopt/install/trajopt_ifopt/lib/libtrajopt_ifopt.so
/opt/tesseract_planning/install/tesseract_motion_planners/lib/libtesseract_motion_planners_trajopt_ifopt.so
/opt/tesseract_planning/install/tesseract_motion_planners/lib/libtesseract_motion_planners_trajopt.so
/opt/tesseract_planning/install/tesseract_task_composer/lib/libtesseract_task_composer.so
/opt/tesseract_planning/install/tesseract_task_composer/lib/libtesseract_task_composer_planning.so
)

install(TARGETS
  tesseract_example_node
  example_traj
  DESTINATION lib/${PROJECT_NAME})

ament_package()
