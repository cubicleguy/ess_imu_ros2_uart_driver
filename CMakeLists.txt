cmake_minimum_required(VERSION 3.5)
project(epson_imu_uart_ros2)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Refer to the README_src.md inside the src folder for more details about IMU macro
# Uncomment the imu_model to build for
# NOTE: Change IMU models require rebuilding binaries with colcon
#set(imu_model "G325PDF0")
#set(imu_model "G325PDF1")
set(imu_model "G365PDF0")
#set(imu_model "G365PDF1")
#set(imu_model "G365PDC0")
#set(imu_model "G365PDC1")
#set(imu_model "G370PDF0")
#set(imu_model "G370PDF1")
#set(imu_model "G354")
#set(imu_model "G364PDCA")
#set(imu_model "G364PDC0")
#set(imu_model "G320")
#set(imu_model "V340")

message([STATUS] "---- Building for IMU Model: ${imu_model}")
add_definitions(-D${imu_model})

# Specify DEBUG macro to enable any debug code by adding "-DDEBUG" in add_definitions()
#add_definitions(-DDEBUG)

# Create filelist macro of C sources for library
set(lib_sources
  src/hcl_gpio.c
  src/hcl_linux.c
  src/hcl_uart.c
  src/sensor_epsonCommon.c
  src/sensor_epsonUart.c
)

# Add IMU model specific C source to filelist macro
if (imu_model STREQUAL "G365PDF0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (imu_model STREQUAL "G365PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (imu_model STREQUAL "G365PDC0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (imu_model STREQUAL "G365PDC1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (imu_model STREQUAL "G370PDF0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (imu_model STREQUAL "G370PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (imu_model STREQUAL "G325PDF0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG325.c)
elseif (imu_model STREQUAL "G325PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG325.c)
elseif (imu_model STREQUAL "G354")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG354.c)
elseif (imu_model STREQUAL "G364PDCA")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG364.c)
elseif (imu_model STREQUAL "G364PDC0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG364.c)
elseif (imu_model STREQUAL "G320")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG320.c)
elseif (imu_model STREQUAL "V340")
  set(lib_sources ${lib_sources}
    src/sensor_epsonV340.c)
else()
  error("**** Invalid IMU Model")
endif()

# Declare static library for Epson IMU functions
add_library(epson_imu_uart_driver_lib STATIC
  ${lib_sources}
)

# Declare C++ executable
add_executable(imu_node src/epson_imu_node.cpp)

# Link Epson IMU C Library to the ROS node
target_link_libraries(imu_node
  epson_imu_uart_driver_lib
)

# Link C++ Library to the ROS node
ament_target_dependencies(imu_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Mark executables and/or libraries for installation
install(
   TARGETS imu_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Mark directories for installation
install(
   DIRECTORY launch DESTINATION share/${PROJECT_NAME}
)

#ament_export_include_directories(include)
#ament_export_dependencies(std_msgs)
ament_package()
