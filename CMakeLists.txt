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
  add_library(gripper SHARED
    src/gripper_driver.cpp
  )
  add_library(camera SHARED
    src/camera_driver.cpp
  )
  add_library(forceSensor SHARED
    src/sensor_driver.cpp
    src/sensor_interface.cpp
  )
  add_library(urdriver SHARED
    src/URDriver/ur_driver.cpp
    src/URDriver/robot_state.cpp
    src/URDriver/robot_state_RT.cpp
    src/URDriver/ur_communication.cpp
    src/URDriver/ur_communication_RT.cpp
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
add_executable(gripper_bin
  src/robotiq_main.cpp
  src/gripper_driver.cpp
)
# Realsense
add_executable(camera_bin
  src/camera_main.cpp
  src/camera_driver.cpp
)
find_package(OpenCV REQUIRED)
target_link_libraries(camera_bin ${OpenCV_LIBS} -lopencv_aruco)
target_link_libraries(camera_bin -lrealsense2)
# Force Sensor
add_executable(forceSensor_bin
  src/sensor_uskin/sensor_driver.cpp
  src/sensor_uskin/sensor_interface.cpp
  src/sensor_uskin/sensor_main.cpp
)
target_link_libraries(forceSensor_bin Ginkgo_Driver)
# Gripper
add_executable(crg_bin
  src/gripper_driver.cpp
  src/gripper_main.cpp
)
# Main
add_executable(ur_bin
  src/main.cpp
  src/system_time.cpp
  src/thread_pool.cpp
  src/data_exchange.cpp
  src/user_interface.cpp
  # src/sensor_driver.cpp
  # src/sensor_interface.cpp
  src/camera_driver.cpp
  src/camera_interface.cpp
  src/ur5e_driver.cpp
  src/ur5e_interface.cpp
  src/gripper_interface.cpp
  src/gripper_driver.cpp
)

# Link the demo_lib target with the demo_bin target
# target_link_libraries(demo_bin PRIVATE
#   demo_lib
# )
# find_package(Threads)
# target_link_libraries(ur_bin ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(ur_bin -pthread)
target_link_libraries(ur_bin ${OpenCV_LIBS} -lopencv_aruco)
target_link_libraries(ur_bin -lrealsense2)
target_link_libraries(ur_bin urdriver gripper camera forceSensor Ginkgo_Driver)

