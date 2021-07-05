#!/bin/bash

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).

# set initial position for each eye-bot

create_bot() {
	cat <<- CREATE_BOT
		    <eye-bot id="ID" rab_range="4">
		      <body position="POSITION" orientation="0,0,0" />
		      <controller config="mavlink_argos_2" />
		    </eye-bot>
	CREATE_BOT
}

INIT_POS_FILE=/tmp/mavlink_argos_2_init_position.txt
> $INIT_POS_FILE

ARGOS_CONFIG_FILE="$HOME/catkin_ws/src/argos_bridge/argos_worlds/mavlink_argos_2.argos"

if [ -e "${ARGOS_CONFIG_FILE}.bak" ]; then
	cp "${ARGOS_CONFIG_FILE}.bak" "$ARGOS_CONFIG_FILE"
else
	cp "$ARGOS_CONFIG_FILE" "${ARGOS_CONFIG_FILE}.bak"
fi

eyebot_position=("-12,-12,0" "-12,12,0" "12,-12,0" "12,12,0" "1,1,0")
#eyebot_position=("-12,-12,0" "12,12,0")
#eyebot_position=("1,1,0")
n=${#eyebot_position[*]}
for (( i=0; i<n; i++ )); do
	id=eyebot_${i}
	create_bot | sed -e "s/ID/$id/" -e "s/POSITION/${eyebot_position[$i]}/" >> eyebots 
	echo "${id}_position=${eyebot_position[$i]}" >> $INIT_POS_FILE
done
sed -n '1,/<arena/p' $ARGOS_CONFIG_FILE > temp_front 
sed -n '\%</arena>%,$p' $ARGOS_CONFIG_FILE > temp_back 
cat temp_front eyebots temp_back > $ARGOS_CONFIG_FILE
rm temp_front eyebots temp_back

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
