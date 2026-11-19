cmake_minimum_required(VERSION 3.16)
project(ptc_core)

option(PTC_STANDALONE "Build core and logic tests without ROS packaging" OFF)
if(NOT PTC_STANDALONE)
  find_package(ament_cmake REQUIRED)
endif()

add_library(${PROJECT_NAME} STATIC
  src/pose_math.cpp
  src/vision/rgbd_image_frame.cpp
  src/vision/bin_height_model.cpp
  src/vision/depth_workspace_model.cpp
  src/vision/point_index.cpp
  src/vision/upright_pose_model.cpp
  src/vision/cloud_registration_model.cpp
  src/vision/grasp_pose_model.cpp
  src/vision/grasp_patch_model.cpp
  src/vision/grasp_surface_model.cpp
  src/config/system_settings.cpp
  src/state/event_context.cpp
  src/state/device_snapshot.cpp
  src/state/track_state.cpp
  src/state/admission_model.cpp
  src/state/motion_state.cpp
  src/state/grasp_state.cpp
  src/state/scene_state.cpp
  src/state/task_state.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /W4 /permissive-)
else()
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
)

if(BUILD_TESTING)
  include(CTest)
  add_executable(arm_contract_test test/arm_contract_test.cpp)
  target_link_libraries(arm_contract_test PRIVATE ${PROJECT_NAME})
  add_test(NAME arm_contract_test COMMAND arm_contract_test)
  add_executable(settings_and_snapshot_test test/settings_and_snapshot_test.cpp)
  target_link_libraries(settings_and_snapshot_test PRIVATE ${PROJECT_NAME})
  add_test(NAME settings_and_snapshot_test COMMAND settings_and_snapshot_test)
  foreach(test_name bin_recovery_policy_test bin_height_test observe_track_test grasp_patch_test grasp_pose_test depth_workspace_test track_motion_test admission_window_test grasp_scene_test full_cycle_test task_fault_test transition_matrix_test edge_cases_test)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE ${PROJECT_NAME})
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()

if(NOT PTC_STANDALONE)
  ament_export_include_directories(include)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_package()
endif()
