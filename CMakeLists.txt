cmake_minimum_required(VERSION 3.5)
project(hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SetRobotArmState.msg"
  "msg/SetServos.msg"
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include/${PROJECT_NAME}/)

# add the ServoDriver library
add_library(ServoDriver SHARED
  src/Servo_LLD/ServoDriver.cpp
  src/Servo_LLD/SerialServoCommunication.cpp)
ament_export_targets(ServoDriver HAS_LIBRARY_TARGET)
ament_export_dependencies(ServoDriver rclcpp std_msgs Boost)

# install ServoDriver
install(
  DIRECTORY include/hardware_interface/Servo_LLD/
  DESTINATION include
)

install(
  TARGETS ServoDriver 
  EXPORT ServoDriver
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# add the executable
add_executable(Demo src/DemoApplication/demo_main.cpp src/DemoApplication/DemoApplication.cpp)
ament_target_dependencies(Demo rclcpp std_msgs)

add_executable(Main_RobotArmDriver src/Robot_arm_HLD/robotarm_main.cpp src/Robot_arm_HLD/RobotArmDriver.cpp)
ament_target_dependencies(Main_RobotArmDriver rclcpp std_msgs)

target_link_libraries(Main_RobotArmDriver ServoDriver)

# Making ROS 2 able to find our package:
install(TARGETS Main_RobotArmDriver DESTINATION lib/${PROJECT_NAME})
install(TARGETS Demo DESTINATION lib/${PROJECT_NAME})



rosidl_target_interfaces(Main_RobotArmDriver ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(Demo ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()