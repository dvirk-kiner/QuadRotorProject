#!/bin/bash


echo "init files"
./init.sh

source devel/setup.bash

echo "Starting script"

FILE="$1" # path to the plan.pddl
xterm -e bash runMainScript.sh -hold &
x=1
while [ ! -s "$FILE" ] # plan.pddl file exists and not empty
do
  echo "Inside while loop"
  sleep 1
  x=$(($x +1))
  if [ "$x" -gt 60 ]; then 
    echo "There is a problem with planning - exit in 5 sec now"
    sleep 5
    exit
  fi
done


echo "Calling Main.py"
#source devel/setup.bash
MAIN_PYTHON="$2"
python "$MAIN_PYTHON"

