cmake_minimum_required(VERSION 3.8)
project(compare_flexbe_utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CartesianMoveToPose.srv"
  "srv/MoveToNamedPose.srv"
  "srv/MoveToPose.srv"
  "srv/VoxelGridFilter.srv"
  "srv/PlaneSegmentation.srv"
  "srv/EuclideanClustering.srv"
  "srv/GetPointCloud.srv"
  "srv/FilterByIndices.srv"
  "action/MTCPlanAndExecutePick.action"
  DEPENDENCIES
    sensor_msgs
    pcl_msgs
    geometry_msgs
    std_msgs
)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

# Component library for the MTC action server
add_library(mtc_plan_and_execute_pick_action_node SHARED
  src/mtc_plan_and_execute_pick_action.cpp
)
target_include_directories(mtc_plan_and_execute_pick_action_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtc_plan_and_execute_pick_action_node
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  moveit_core
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
)

# Register the component (plugin string must match your C++ namespace::Class)
rclcpp_components_register_nodes(
  mtc_plan_and_execute_pick_action_node
  "compare_flexbe_utilities::MtcPlanAndExecutePickActionNode"
)

# Create executable
add_executable(cartesian_move_to_pose_service src/cartesian_move_to_pose_service.cpp)
ament_target_dependencies(cartesian_move_to_pose_service
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(move_to_named_pose_service src/move_to_named_pose_service.cpp)
ament_target_dependencies(move_to_named_pose_service
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(move_to_pose_service src/move_to_pose_service.cpp)
ament_target_dependencies(move_to_pose_service
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(voxel_grid_filter_service src/voxel_grid_filter_service.cpp)
ament_target_dependencies(voxel_grid_filter_service
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(voxel_grid_filter_service ${PCL_LIBRARIES})

add_executable(plane_segmentation_service src/plane_segmentation_service.cpp)
ament_target_dependencies(plane_segmentation_service
  rclcpp
  pcl_conversions
  pcl_msgs
)
target_link_libraries(plane_segmentation_service ${PCL_LIBRARIES})

add_executable(euclidean_clustering_service src/euclidean_clustering_service.cpp)
ament_target_dependencies(euclidean_clustering_service
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_msgs
)
target_link_libraries(euclidean_clustering_service ${PCL_LIBRARIES})

add_executable(get_point_cloud_service src/get_point_cloud_service.cpp)
ament_target_dependencies(get_point_cloud_service
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)
target_link_libraries(get_point_cloud_service ${PCL_LIBRARIES})

add_executable(filter_by_indices_service src/filter_by_indices_service.cpp)
ament_target_dependencies(filter_by_indices_service
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)
target_link_libraries(filter_by_indices_service ${PCL_LIBRARIES})

add_executable(mtc_plan_and_execute_pick_action_main  src/mtc_plan_and_execute_pick_action_main.cpp)
ament_target_dependencies(mtc_plan_and_execute_pick_action_main
  rclcpp
)
target_link_libraries(mtc_plan_and_execute_pick_action_main mtc_plan_and_execute_pick_action_node)

install(TARGETS 
  cartesian_move_to_pose_service
  move_to_named_pose_service
  move_to_pose_service
  voxel_grid_filter_service
  plane_segmentation_service
  euclidean_clustering_service
  get_point_cloud_service
  filter_by_indices_service
  mtc_plan_and_execute_pick_action_node
  mtc_plan_and_execute_pick_action_main
  DESTINATION lib/${PROJECT_NAME})

# Link generated service headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(cartesian_move_to_pose_service ${cpp_typesupport_target})
target_link_libraries(move_to_named_pose_service ${cpp_typesupport_target})
target_link_libraries(move_to_pose_service ${cpp_typesupport_target})
target_link_libraries(voxel_grid_filter_service ${cpp_typesupport_target})
target_link_libraries(plane_segmentation_service ${cpp_typesupport_target})
target_link_libraries(euclidean_clustering_service ${cpp_typesupport_target})
target_link_libraries(get_point_cloud_service ${cpp_typesupport_target})
target_link_libraries(filter_by_indices_service ${cpp_typesupport_target})
target_link_libraries(mtc_plan_and_execute_pick_action_node ${cpp_typesupport_target})
target_link_libraries(mtc_plan_and_execute_pick_action_main ${cpp_typesupport_target})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_dependencies(rosidl_default_runtime)
ament_package()
