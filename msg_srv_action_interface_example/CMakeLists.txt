cmake_minimum_required(VERSION 3.5)
project(msg_srv_action_interface_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files "msg/ArithmeticArgument.msg")
set(srv_files "srv/ArithmeticOperator.srv")
set(action_files "action/ArithmeticChecker.action")

set(buffer_msg_files "msg/Buffer.msg")
set(communication_buffer_srv_files "srv/CommunicationBuffer.srv")
set(append_srv "srv/Append.srv")
set(process_srv "srv/Process.srv")
set(core_to_slave_srv "srv/CoreToSlave.srv")
set(enrollment_srv "srv/Enrollment.srv")
set(send_to_gui "srv/SendToGUI.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  ${buffer_msg_files}
  ${communication_buffer_srv_files}
  ${append_srv}
  ${process_srv}
  ${core_to_slave_srv}
  ${enrollment_srv}
  ${send_to_gui}
  DEPENDENCIES builtin_interfaces
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
