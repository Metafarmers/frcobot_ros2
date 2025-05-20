cmake_minimum_required(VERSION 3.8)
project(fairino_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(fairino_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# 소스 파일 설정
file(GLOB SRC_DIR ./src/command_server.cpp ./src/command_server_node.cpp)
file(GLOB EXP_SRC_DIR ./examples/src/*.cpp)
file(GLOB XMLRPC_SRC ${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc/*.cpp)

# 실행 파일
add_executable(ros2_cmd_server ${SRC_DIR})
add_executable(srv_test_client ${EXP_SRC_DIR})

# 라이브러리 정의 및 소스 연결
add_library(fairino_hardware SHARED)
target_sources(fairino_hardware PRIVATE
  ./src/fairino_robot.cpp 
  ./src/fairino_hardware_interface.cpp
  ${XMLRPC_SRC}
)

# include 경로 설정
target_include_directories(ros2_cmd_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(srv_test_client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/examples/include)

target_include_directories(fairino_hardware PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc>
  $<INSTALL_INTERFACE:include/fairino_hardware>
  $<INSTALL_INTERFACE:include/xmlrpc>
)

# ament dependencies
ament_target_dependencies(ros2_cmd_server rclcpp fairino_msgs)
ament_target_dependencies(srv_test_client rclcpp fairino_msgs)
ament_target_dependencies(fairino_hardware 
  hardware_interface
  pluginlib
  rclcpp
  fairino_msgs
  rclcpp_lifecycle 
)

# pluginlib export
pluginlib_export_plugin_description_file(hardware_interface fairino_hardware.xml)

# 설치 설정
install(TARGETS ros2_cmd_server
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS srv_test_client
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS fairino_hardware
  EXPORT fairino_hardware
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# include 디렉토리 설치
install(DIRECTORY include/fairino_hardware
  DESTINATION include
)

install(DIRECTORY include/xmlrpc
  DESTINATION include
)

# 테스트 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)

  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# export 설정
ament_export_targets(fairino_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp hardware_interface pluginlib rclcpp_lifecycle)

ament_package()
