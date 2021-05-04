#!/bin/bash

# The purpose of this script is to create a launch file for a multi-robot
# system by replicating a "group" tag "n" times, then executing the resulting
# launch file.

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).
n=5

LAUNCH_FILE=/tmp/mavlink_argos.launch

echo "<launch>" > $LAUNCH_FILE

for ((i=0; i<n; i++)); do
    namespace="eyebot$i"
    echo -e "\t<group ns=\"$namespace\">"
    echo -e "\t\t<node pkg=\"argos_bridge\" type=\"mavlink_argos_controller\" name=\"mavlink_argos_controller\" output=\"screen\" />"
    echo -e "\t</group>"
done >> $LAUNCH_FILE
echo -e "</launch>" >> $LAUNCH_FILE

roslaunch $LAUNCH_FILE
