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
function(build_driver)
  add_library(robotiq SHARED
    src/robotiq_driver.cpp
  )
  add_library(camera SHARED
    src/camera_driver.cpp
  )
  add_library(forceSensor SHARED
    src/sensor_driver.cpp
    src/sensor_interface.cpp
  )
  add_library(ur5e SHARED
    src/UR5eDriver/ur_driver.cpp
    src/UR5eDriver/robot_state.cpp
    src/UR5eDriver/robot_state_RT.cpp
    src/UR5eDriver/ur_communication.cpp
    src/UR5eDriver/ur_communication_RT.cpp
  )
endfunction(build_driver)
if(BUILD_NEW_DRIVER)
  message("==>> Build dynamic link library.\n")
  build_driver()
elseif(EXISTS "${CMAKE_SOURCE_DIR}/lib/libur5e.so")
  message("==>> Use existing .so file.\n")
else()
  message("==>> Lack of .so file, build now.\n")
  build_driver()
endif()
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
# RobotiQ
add_executable(robotiq_bin
  src/robotiq_main.cpp
  src/robotiq_driver.cpp
)
# Realsense
add_executable(camera_bin
  src/camera_main.cpp
  src/camera_driver.cpp
  src/user_interface.cpp
)
find_package(OpenCV REQUIRED)
target_link_libraries(camera_bin ${OpenCV_LIBS})
target_link_libraries(camera_bin -lrealsense2)
# Force Sensor
add_executable(forceSensor_bin
  src/sensor_driver.cpp
  src/sensor_interface.cpp
  src/sensor_main.cpp
)
target_link_libraries(forceSensor_bin Ginkgo_Driver)
# Main
add_executable(ur_bin
  src/main.cpp
  src/trajectory.cpp
  src/system_time.cpp
  src/thread_pool.cpp
  src/ur_kinematics.cpp
  src/path_planning.cpp
  src/data_exchange.cpp
  src/user_interface.cpp
  src/sensor_driver.cpp
  src/sensor_interface.cpp
)

# Link the demo_lib target with the demo_bin target
# target_link_libraries(demo_bin PRIVATE
#   demo_lib
# )
find_package(Threads)
target_link_libraries(ur_bin ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(ur_bin ur5e robotiq camera forceSensor Ginkgo_Driver)

