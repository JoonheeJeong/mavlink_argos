add_library(mavlink_argos MODULE mavlink_argos.h mavlink_argos.cpp)
target_link_libraries(mavlink_argos
  argos3core_simulator
  argos3plugin_simulator_eyebot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

