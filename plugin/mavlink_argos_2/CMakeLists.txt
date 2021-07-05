add_library(mavlink_argos_2 MODULE mavlink_argos_2.h mavlink_argos_2.cpp)
target_link_libraries(mavlink_argos_2
  argos3core_simulator
  argos3plugin_simulator_eyebot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

