cmake_minimum_required(VERSION 3.8)
project(mod101_hw_bringup)

find_package(ament_cmake REQUIRED)

# Config and launch only — every node this package starts comes from somewhere
# else. That is the point: this is the robot's configuration, not its code.
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
