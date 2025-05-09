cmake_minimum_required(VERSION 3.8)
project(mission)

# Enable compiler warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED) # Uncomment if used

# ------------------------------------------------------------------------------
# Install Python executable scripts
# ------------------------------------------------------------------------------
install(PROGRAMS
  scripts/mission.py
  scripts/aruco_tracker.py
  scripts/spiral_trajectory.py
  scripts/estimate.py
  scripts/landing.py
  scripts/return.py
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Install directories (e.g., launch, config, models, scripts)
# ------------------------------------------------------------------------------
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Package manifest
# ------------------------------------------------------------------------------
ament_package()
