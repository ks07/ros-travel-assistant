#!/bin/bash -x

# Hide debugging output from the ROS environment files
set +x

source /opt/ros/indigo/setup.bash
source ~/catkin_ws_3/devel/setup.bash

set -x

roscd wheely_sim
(roscore & echo $! >> /tmp/mainpids) &
# Should run stage here but CrossRoadServer needs improvement

COUNTER=1
while [ $COUNTER -lt 5 ]; do
	sleep 7
	rm -f /tmp/rospids
	# Need to set parameters
	rosrun wheely_sim set_test_params.py
	# Run simulator
	(rosrun stage_ros stageros road-crossing-island.world & echo $! >> /tmp/rospids) &
	# Run functional coverage collector
	(rosrun wheely_sim functional_cov_collector.py & echo $! >> /tmp/rospids; mv .fcov .fcov_$COUNTER) &
	# Run robot code
	(COVERAGE_FILE=.coverage_crs_$COUNTER rosrun wheely_sim CrossRoadServer.py & echo $! >> /tmp/rospids) &
	(COVERAGE_FILE=.coverage_wly_$COUNTER rosrun wheely_sim wheely.py; ) &
	# Run the assertion monitors
	(rosrun wheely_sim monitor1.py & echo $! >> /tmp/rospids) &
	(rosrun wheely_sim monitor2.py & echo $! >> /tmp/rospids) &
	(rosrun wheely_sim monitor3.py & echo $! >> /tmp/rospids) &
	(rosrun wheely_sim monitor5.py & echo $! >> /tmp/rospids) &
	# Run the test drivers
	(rosrun wheely_sim bdi_interface.py jason/wheely/bdi_tests/user.txt & echo $! >> /tmp/rospids) &
	(sleep 10 && echo '' | rosrun wheely_sim bdi_interface.py jason/wheely/bdi_tests/lights.txt -t & echo $! >> /tmp/rospids)

	# Need to wait for the tests to be over, can (ab)use rostopic
	rostopic echo -n 1 --filter='m.data ==127' /user_commands
	
	# Give a few seconds for proper shutdown, repeat the command just in case
	rostopic pub -1 /user_commands std_msgs/Int8 127
	sleep 5

	# Ask nicely
	cat /tmp/rospids | xargs kill
	
	sleep 5

	# Ask nastily
	cat /tmp/rospids | xargs kill -9

	
        let COUNTER=COUNTER+1 
done

cat /tmp/mainpids | xargs kill

# Combine the code coverage results
coverage combine .coverage_*

# Regenerate the html output
coverage html

echo "Testbench Done"
