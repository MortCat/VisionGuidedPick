# RoboStack stores non-ROS native libraries below CONDA_PREFIX/Library.
if(WIN32 AND DEFINED ENV{CONDA_PREFIX})
  list(PREPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}/Library")
endif()
