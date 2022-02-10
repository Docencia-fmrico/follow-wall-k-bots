cmake_minimum_required(VERSION 3.5)
project(follow-wall)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_copyright
    )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_cmake_gtest)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_add_gtest(node_test test/node_test.cpp)
target_link_libraries(node_test ${PROJECT_NAME})

set(dependencies
    rclcpp
    rclcpp_lifecycle
    std_msgs
    sensor_msgs
    geometry_msgs 
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
src/follow_wall/follow_wall.cpp
)

add_executable(follow_wall_main src/follow_wall_main.cpp)
ament_target_dependencies(follow_wall_main ${dependencies})

target_link_libraries(follow_wall_main ${PROJECT_NAME})

install(TARGETS
  follow_wall_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
DESTINATION include/
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



ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

ament_package()
