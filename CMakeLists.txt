cmake_minimum_required(VERSION 3.14)
project(boreas)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

set(BOREAS_SOURCES
  src/boreas_node.cpp
  src/boreas_parameters.cpp
  src/path_utils.cpp
  src/dataset_index.cpp
  src/sensor_loaders.cpp
  src/calibration.cpp
  src/static_transforms.cpp
  src/map_projector.cpp
)

ament_auto_add_executable(${PROJECT_NAME} ${BOREAS_SOURCES})

ament_target_dependencies(${PROJECT_NAME}
  autoware_geography_utils
  autoware_map_projection_loader
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

ament_auto_package(INSTALL_TO_SHARE launch config)
