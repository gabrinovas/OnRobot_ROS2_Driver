cmake_minimum_required(VERSION 3.5)
project(onrobot_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/include/onrobot_driver
  ${rclcpp_INCLUDE_DIRS}
)

# Enable serial communication for the Modbus library.
set(MODBUS_SERIAL_COMMUNICATION ON CACHE BOOL "Enable Modbus Serial Communication" FORCE)

# Add the Modbus library as a subdirectory.
add_subdirectory(include/onrobot_driver/Modbus)

add_executable(onrobot_driver_node
  src/onrobot_driver_node.cpp
  src/RG.cpp
  include/onrobot_driver/TCPConnectionWrapper.hpp
  include/onrobot_driver/SerialConnectionWrapper.hpp
  include/onrobot_driver/IModbusConnection.hpp
)

target_link_libraries(onrobot_driver_node
  ${rclcpp_LIBRARIES}
  Modbus
)

install(TARGETS onrobot_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
