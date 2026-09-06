cmake_minimum_required(VERSION 3.0.2)
project(garbage_localization)

find_package(PythonInterp REQUIRED)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  vision_msgs
)

add_message_files(
  FILES
  GarbageObject.msg
  GarbageObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES garbage_localization_utils
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs message_runtime roscpp sensor_msgs
                 std_msgs tf2 tf2_geometry_msgs tf2_ros vision_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(garbage_localization_utils
  src/localization_utils.cpp
)
add_dependencies(garbage_localization_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(garbage_localization_utils ${catkin_LIBRARIES})

add_executable(garbage_localization_node
  src/garbage_localization_node.cpp
)
add_dependencies(garbage_localization_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(garbage_localization_node
  garbage_localization_utils
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(localization_utils_test
    test/localization_utils_test.cpp
  )
  if(TARGET localization_utils_test)
    target_link_libraries(localization_utils_test
      garbage_localization_utils
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_nosetests(test/test_multi_bottle_evaluation.py)
  catkin_add_nosetests(test/test_robocup_task_flow.py)
  catkin_add_nosetests(test/test_sequential_navigation_regression.py)
endif()

install(TARGETS garbage_localization_utils garbage_localization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/evaluate_multi_bottle.py
  scripts/multi_bottle_evaluation_utils.py
  scripts/run_robocup_baseline.py
  scripts/run_robocup_task_flow.py
  scripts/run_sequential_navigation_regression.py
  scripts/run_four_paper_ball_fresh_regression.py
  scripts/calibrate_four_paper_ball_scene.py
  scripts/paper_ball_map_stabilizer.py
  scripts/sample_localization_stability.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
