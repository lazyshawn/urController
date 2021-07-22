############################################################################
# ==>> Basic Setting
############################################################################
cmake_minimum_required(VERSION 3.5)
# Project name | ${PROJECT_NAME}
project(urcontroller)
set(CMAKE_CXX_STANDARD 17)

############################################################################
# ==>> Generate a library
############################################################################
# Output of the library
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
# Generate the library from the library sources
add_library(robotiq SHARED
  src/robotiq.cpp
)
add_library(camera SHARED
  src/camera.cpp
)
# Specify include directories for a lib
# target_include_directories(demo_lib PUBLIC
#   ${PROJECT_SOURCE_DIR}/include
# )

############################################################################
# ==>> Create an executable
############################################################################
# Source directories
# aux_source_directory(${CMAKE_SOURCE_DIR}/src
#   DIR_SRCS
# )
# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
# Link directories
link_directories(${CMAKE_SOURCE_DIR}/lib)
# Add an executable with the above sources
add_executable(ur_bin
  src/main.cpp
  src/ur_driver.cpp
  src/trajectory.cpp
  src/system_time.cpp
  src/robot_state.cpp
  src/thread_pool.cpp
  src/ur_kinematics.cpp
  src/path_planning.cpp
  src/data_exchange.cpp
  src/robot_state_RT.cpp
  src/user_interface.cpp
  src/ur_communication.cpp
  src/ur_communication_RT.cpp
)
# RobotiQ
add_executable(robotiq_bin
  src/main_robotiq.cpp
  src/robotiq.cpp
)
# Realsense
add_executable(camera_bin
  src/main_camera.cpp
  src/camera.cpp
  src/user_interface.cpp
)

# Link the demo_lib target with the demo_bin target
# target_link_libraries(demo_bin PRIVATE
#   demo_lib
# )
find_package(OpenCV REQUIRED)
target_link_libraries(camera_bin ${OpenCV_LIBS})
target_link_libraries(camera_bin -lrealsense2)
find_package(Threads)
target_link_libraries(ur_bin ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(ur_bin robotiq camera)

