cmake_minimum_required(VERSION 3.14)
project(boreas)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)


include_directories(${CMAKE_SOURCE_DIR}/include)

ament_auto_add_executable(${PROJECT_NAME}
  src/boreas_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES} yaml-cpp
)

ament_auto_package(INSTALL_TO_SHARE launch config)

