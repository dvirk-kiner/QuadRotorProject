#!/bin/bash
source devel/setup.bash
echo "Launching Takeoff"
rostopic pub /ardrone/takeoff std_msgs/Empty "{}"
