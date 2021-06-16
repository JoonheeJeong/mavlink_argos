#!/bin/bash

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).
n=5

# set initial position for each eye-bot

ARGOS_CONFIG_FILE="$HOME/catkin_ws/src/argos_bridge/argos_worlds/mavlink_argos_2.argos"
echo "ARGOS_CONFIG_FILE=$ARGOS_CONFIG_FILE"
if [ -e "${ARGOS_CONFIG_FILE}.bak" ]; then
	cp "${ARGOS_CONFIG_FILE}.bak" "$ARGOS_CONFIG_FILE"
else
	cp "$ARGOS_CONFIG_FILE" "${ARGOS_CONFIG_FILE}.bak"
fi

INIT_POS_FILE=/tmp/mavlink_argos_2_init_position.txt
> $INIT_POS_FILE

eyebot_position=("-12,-12,0" "-12,12,0" "12,-12,0" "12,12,0" "1,1,0")
for (( i=0; i<n; i++ )); do
	str=eyebot_${i}_position
	echo "str=$str"
	sed -i "s/$str/${eyebot_position[$i]}/" $ARGOS_CONFIG_FILE
	echo "$str=${eyebot_position[$i]}" >> $INIT_POS_FILE
done


# create a launch file for a multi-robot system 
# by replicating a "group" tag "n" times, 
# then executing the resulting launch file.

LAUNCH_FILE=/tmp/mavlink_argos_2.launch

echo "<launch>" > $LAUNCH_FILE

for ((i=0; i<n; i++)); do
    namespace="eyebot_$i"
    echo -e "\t<group ns=\"$namespace\">"
    echo -e "\t\t<node pkg=\"argos_bridge\" type=\"mavlink_argos_2_controller\" name=\"mavlink_argos_2_controller\" output=\"screen\" />"
    echo -e "\t</group>"
done >> $LAUNCH_FILE
echo -e "</launch>" >> $LAUNCH_FILE

roslaunch $LAUNCH_FILE
