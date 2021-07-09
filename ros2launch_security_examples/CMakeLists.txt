cmake_minimum_required(VERSION 3.8)
project(ros2launch_security_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ament_nodl REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(fake_imu_library SHARED src/fake_imu.cpp)
ament_target_dependencies(fake_imu_library rclcpp rclcpp_components sensor_msgs example_interfaces)
rclcpp_components_register_node(fake_imu_library
  PLUGIN "ros2launch_security_examples::FakeImu"
  EXECUTABLE fake_imu
)
# TODO(wjwwood): go back to organizing these into a nodl folder when this is fixed:
#   https://github.com/ubuntu-robotics/nodl/issues/41
# nodl_export_node_description_file(nodl/fake_imu.nodl.xml)
nodl_export_node_description_file(fake_imu.nodl.xml)

add_library(imu_sink_library SHARED src/imu_sink.cpp)
ament_target_dependencies(imu_sink_library rclcpp rclcpp_components sensor_msgs example_interfaces)
rclcpp_components_register_node(imu_sink_library
  PLUGIN "ros2launch_security_examples::ImuSink"
  EXECUTABLE imu_sink
)
# TODO(wjwwood): go back to organizing these into a nodl folder when this is fixed:
#   https://github.com/ubuntu-robotics/nodl/issues/41
# nodl_export_node_description_file(nodl/imu_sink.nodl.xml)
nodl_export_node_description_file(imu_sink.nodl.xml)

install(
  TARGETS
    fake_imu_library
    imu_sink_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  get_default_rmw_implementation(default_rmw_implementation)

  add_launch_test(
    "test/test_fake_imu_launch.py"
    TARGET test_fake_imu_launch
    TIMEOUT 60
    ENV
      RCL_ASSERT_RMW_ID_MATCHES=${default_rmw_implementation}
      RMW_IMPLEMENTATION=${default_rmw_implementation}
  )

  add_launch_test(
    "test/test_fake_imu_launch_with_security.py"
    TARGET test_fake_imu_launch_with_security
    TIMEOUT 60
    ENV
      RCL_ASSERT_RMW_ID_MATCHES=${default_rmw_implementation}
      RMW_IMPLEMENTATION=${default_rmw_implementation}
  )

  add_launch_test(
    "test/test_fake_imu_launch_with_security_nodltopolicy.py"
    TARGET test_fake_imu_launch_with_security_nodl_to_policy
    TIMEOUT 60
    ENV
      RCL_ASSERT_RMW_ID_MATCHES=${default_rmw_implementation}
      RMW_IMPLEMENTATION=${default_rmw_implementation}
  )

  set_tests_properties(
    test_fake_imu_launch
    test_fake_imu_launch_with_security
    test_fake_imu_launch_with_security_nodl_to_policy
    PROPERTIES
      DEPENDS fake_imu
      DEPENDS imu_sink
  )
endif()

ament_package()
