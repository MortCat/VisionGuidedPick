cmake_minimum_required(VERSION 3.16)
project(ptc_mock)

find_package(ament_cmake REQUIRED)
find_package(ptc_motion REQUIRED)
add_executable(arm_gateway_mock src/arm_gateway_main.cpp src/mock_motion_backend.cpp)
target_compile_features(arm_gateway_mock PRIVATE cxx_std_17)
target_include_directories(arm_gateway_mock PRIVATE include)
target_link_libraries(arm_gateway_mock PRIVATE ptc_motion::ptc_motion)
install(TARGETS arm_gateway_mock RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
