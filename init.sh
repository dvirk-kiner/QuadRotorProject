#!/bin/bash

res_file="src/rotor_control/result.txt"
[ -e $res_file ] && rm $res_file
rm ./src/rotor_control/scripts/result.txt 2>/dev/null
cp -R ./src/rotor_control/scripts/photos_taken_by_quadrotor/InTesting/tmp/* ./src/rotor_control/scripts/photos_taken_by_quadrotor/PreTesting 2>/dev/null
cp -R ./src/rotor_control/scripts/photos_taken_by_quadrotor/Done/* ./src/rotor_control/scripts/photos_taken_by_quadrotor/PreTesting 2>/dev/null
rm -r ./src/rotor_control/scripts/photos_taken_by_quadrotor/InTesting/tmp/* 2>/dev/null
rm -r ./src/rotor_control/scripts/photos_taken_by_quadrotor/Done/* 2>/dev/null

rm ./src/rotor_control/common/domain.pddl 2>/dev/null
rm ./src/rotor_control/common/problem.pddl 2>/dev/null
rm ./src/rotor_control/common/planRendered.txt 2>/dev/null
rm ./src/rotor_control/common/plan.pddl 2>/dev/null

cp ./src/rotor_control/common/new/* ./src/rotor_control/common



