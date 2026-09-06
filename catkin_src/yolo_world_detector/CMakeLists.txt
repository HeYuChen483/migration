cmake_minimum_required(VERSION 3.0.2)
project(yolo_world_detector)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  roslib
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  vision_msgs
)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)

set(ONNXRUNTIME_ROOT "/home/hyc/robocup_vision/onnxruntime" CACHE PATH
    "Path to the ONNX Runtime C++ SDK")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS "$ENV{ONNXRUNTIME_ROOT}" "${ONNXRUNTIME_ROOT}"
  PATH_SUFFIXES include)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  HINTS "$ENV{ONNXRUNTIME_ROOT}" "${ONNXRUNTIME_ROOT}"
  PATH_SUFFIXES lib lib64)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR
    "ONNX Runtime C++ SDK not found. Set ONNXRUNTIME_ROOT or pass "
    "-DONNXRUNTIME_ROOT=/path/to/onnxruntime")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yolo_world_utils
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp roslib sensor_msgs tf2 tf2_geometry_msgs tf2_ros vision_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

add_library(yolo_world_utils src/yolo_world_utils.cpp)
target_link_libraries(yolo_world_utils ${OpenCV_LIBRARIES})

add_executable(yolo_world_detector_node src/yolo_world_detector_node.cpp)
add_dependencies(yolo_world_detector_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(yolo_world_detector_node
  yolo_world_utils
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ONNXRUNTIME_LIBRARY}
)

add_executable(enhanced_detection_merger_node src/enhanced_detection_merger_node.cpp)
add_dependencies(enhanced_detection_merger_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(enhanced_detection_merger_node
  ${catkin_LIBRARIES}
)

add_executable(bytetrack_node src/bytetrack_node.cpp)
add_dependencies(bytetrack_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(bytetrack_node
  ${catkin_LIBRARIES}
)

add_executable(paper_ball_rgbd_vote_node src/paper_ball_rgbd_vote_node.cpp)
add_dependencies(paper_ball_rgbd_vote_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(paper_ball_rgbd_vote_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(detection_overlay_node src/detection_overlay_node.cpp)
add_dependencies(detection_overlay_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(detection_overlay_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
set_target_properties(yolo_world_detector_node PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
  INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(yolo_world_utils_test test/yolo_world_utils_test.cpp)
  if(TARGET yolo_world_utils_test)
    target_link_libraries(yolo_world_utils_test yolo_world_utils ${OpenCV_LIBRARIES})
  endif()
endif()

install(TARGETS yolo_world_utils yolo_world_detector_node enhanced_detection_merger_node bytetrack_node paper_ball_rgbd_vote_node detection_overlay_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/sim_trash_bin_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
