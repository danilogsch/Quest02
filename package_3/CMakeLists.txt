cmake_minimum_required(VERSION 3.8)
project(package_3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Gerar a interface de ação
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FindPrime.action"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Dependências para os alvos
add_executable(prime_action_server src/prime_action_server.cpp)
ament_target_dependencies(prime_action_server rclcpp rclcpp_action)
target_link_libraries(prime_action_server ${cpp_typesupport_target})
add_executable(prime_action_client src/prime_action_client.cpp)
ament_target_dependencies(prime_action_client rclcpp rclcpp_action)
target_link_libraries(prime_action_client ${cpp_typesupport_target})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Instalação
install(TARGETS
  prime_action_server
  prime_action_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
