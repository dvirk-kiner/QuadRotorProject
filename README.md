# Quadrotor-Project

## System environment:

- OS: ubuntu 16.4
- Gazebo simulator: ver 7.16.0
- ROS: Kinetic
- Ros Packages:
- RosPlan: ver. June 2018
- Tum_Simulator
- Rotor_Control (our package)
- Python: ver. 2.7

---

## How to install:

### MODEL WEIGHTS:

Download weights from:
https://technionmail-my.sharepoint.com/:u:/g/personal/dvirh_campus_technion_ac_il/EQF8Vy6U3DtKrs31kuydLcQBTFFsNGRQevdqaUaKciIajg?e=uhSMYD

Run this code:

```
mkdir -p <workspace_folder>/src/rotor_control/scripts/ML/saved_models/

cp <downloaded weights path> <workspace_folder>/src/rotor_control/scripts/ML/saved_models/
```

### ROSPLAN:

```
cd <workspace_folder>/src

git clone https://github.com/KCL-Planning/ROSPlan

cd ..
```

### TUM_SIMULATOR:

```
sudo apt-get install ros-kinetic-hector-*

sudo apt-get install ros-kinetic-ardrone-autonomy
```

### BUILD:

```
cd <workspace_folder>

catkin_make
```

### ADDDIONAL ENVIOURMENT SETTINGS:

```
cd <workspace_folder>/src

mkdir -p ROSPlan/rosplan_knowledge_base/common/mongoDB

mkdir -p rotor_control/scripts/photos_taken_by_quadrotor/InTesting/tmp

pip2 install torch torchvision

pip2 install future
```

### RUNNING SIMULATION:

In a new terminal type:

```
roscore
```

In a diffrent terminal type:

```
source <workspace_folder>/devel/setup.bash

./runAll.sh src/rotor_control/common/plan.pddl src/rotor_control/scripts/mainScript.py
```

### IMPORTATNT NOTES:

- If you want to rename the folder (i.e from "QuadrotorProject" to "catkin_ws" ) make sure to do that **before** build.
  Renaming the folder after catkin_make might cause enviroumnet issues.
- If it's the first time you are running the project, you might get an error and gazebo will be stuck.
  If that happen- **just wait** several minutes for the gezebo to open (without pressing any keys).
  after gazebo opens you can re-run the script and it should work.

### Thanks

Thank you for @ dvirhalutz for a great partnership
