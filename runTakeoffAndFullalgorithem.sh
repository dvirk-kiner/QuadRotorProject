#!/bin/bash
source devel/setup.bash
echo "Running Main Python Script"
rostopic pub /ardrone/takeoff std_msgs/Empty "{}"
python main.py
