# Prerequisites
This assumes you have ROS2 installed on your system. Development was completed using [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html).

The instructions also assume you have a Python virtual environment manager installed. This repo uses [`mamba`](https://mamba.readthedocs.io/en/latest/installation/mamba-installation.html), but you may use `conda` equivalently.

# Create workspace, clone repositories

Create a workspace and clone the ROS packages. At a terminal:
```
cd ~
mkdir -p c2hi_ws/src && cd c2hi_ws/src
git clone https://github.com/UTNuclearRobotics/c2hi -b devel
git clone --recursive https://github.com/roboav8r/mm_scene_rec -b devel
git clone https://github.com/roboav8r/marmot -b devel --depth 1
git clone https://github.com/roboav8r/tracking_msgs -b devel
git clone https://github.com/roboav8r/ar_track_alvar_msgs -b ros2
git clone https://github.com/roboav8r/ros_audition -b devel
git clone https://github.com/ros-drivers/audio_common -b ros2
git clone https://github.com/roboav8r/situated_hri_interfaces -b devel
```

# Create the virtual environment
```
cd ~/c2hi_ws/src/c2hi
mamba env create -f c2hi_env.yml
```

# Building the project
Install ROS dependencies. At a terminal:
```
cd ~/c2hi_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
```

Build the repo. At a terminal:
```
cd ~/c2hi_ws
mamba activate c2hi
colcon build --packages-select audio_common_msgs && source install/setup.bash
colcon build --packages-select situated_hri_interfaces tracking_msgs ar_track_alvar_msgs && source install/setup.bash
colcon build --packages-select ros_audition mm_scene_rec marmot c2hi && source install/setup.bash
```
