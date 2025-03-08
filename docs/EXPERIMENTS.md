# Running the experiments

## Download the evaluation dataset
NOTE: Depending on your internet connection speed, this may take several days to download.

At a terminal, run the script to download the evaluation dataset:
```
cd ~
python3 ~/c2hi_ws/src/c2hi/scripts/download_dataset.py
```
NOTE: By default, this will begin downloading the datafiles to `~/storage/datasets/`.

Next, create a softlink from the project directory to the storage directory. At a terminal:
```
cd ~/c2hi_ws/src/c2hi/bags
ln -s ~/storage/datasets/doi-10-18738-T8-KENJXS exp_data
```

## Experiment 0: Compute observation models

### Experiment 0a: Compute audio and visual scene recognition models (~16 hours)
Launch the ROS nodes
```
cd ~/c2hi_ws
mamba activate c2hi
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch c2hi exp0a_scene_rec_nodes.launch.py
```

In a separate window, run the message playback/model computation script
```
cd ~/c2hi_ws
mamba activate c2hi
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ~/c2hi_ws/src/c2hi/scripts/experiments
python3 exp0a_compute_scene_rec_models.py
```

### Experiment 0b: Compute observation model for visual role recognition (~12 hours)
Run the data playback/role observation model computation script. At a terminal:
```
cd ~/c2hi_ws
mamba activate c2hi
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ~/c2hi_ws/src/c2hi/scripts/experiments
python3 exp0b_compute_role_rec_models.py
```

Run the evaluation script. At a terminal:
```
cd ~/c2hi_ws/src/c2hi/scripts/analysis
python3 exp0b_analyze_role_rec_obs_models.py
```

### Experiment 0c: Evaluate speech recognition parameters (~4 minutes per configuration)
Run the playback/model computation script
```
cd ~/c2hi_ws
mamba activate c2hi
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ~/c2hi_ws/src/c2hi/scripts/experiments
python3 exp0c_evaluate_speech_rec.py
```

## Experiment 1: Running scene recognition experiments (~18 hours)
Run the ROS nodes
```
mamba activate c2hi
source /opt/ros/humble/setup.bash
cd ~/c2hi_ws
colcon build --packages-select ar_track_alvar_msgs audio_common_msgs situated_hri_interfaces tracking_msgs && source install/setup.bash # build messages and interfaces
colcon build --packages-select marmot mm_scene_rec ros_audition c2hi && source install/setup.bash
ros2 launch c2hi exp1_record_scene_results.launch.py
```

## Experiment 2: Running role recognition experiments (~6 hours)
Launch the Docker container. In a terminal:
```
cd ~/c2hi_ws/src/c2hi/docker/experiments
docker compose up
```

Run the ROS nodes. In a second terminal:
```
mamba activate c2hi
source /opt/ros/humble/setup.bash
cd ~/c2hi_ws
colcon build --packages-select ar_track_alvar_msgs audio_common_msgs && source install/setup.bash
colcon build --packages-select ar_track_alvar_msgs audio_common_msgs situated_hri_interfaces tracking_msgs && source install/setup.bash # build messages and interfaces 
colcon build --packages-select marmot mm_scene_rec ros_audition c2hi && source install/setup.bash
ros2 launch c2hi exp2_record_role_results.launch.py
```
## Experiment 3: Running command recognition experiments
Launch the Docker container. In a terminal:
```
cd ~/c2hi_ws/src/c2hi/docker/experiments
docker compose up
```

Run the ROS nodes
```
mamba activate c2hi
source /opt/ros/humble/setup.bash
cd ~/c2hi_ws
colcon build --packages-select ar_track_alvar_msgs audio_common_msgs situated_hri_interfaces tracking_msgs && source install/setup.bash
colcon build --packages-select marmot mm_scene_rec ros_audition gesture_recognition_ros2 c2hi && source install/setup.bash
ros2 launch c2hi exp3a_record_ar_cmd_results.launch.py
ros2 launch c2hi exp3b_record_verbal_cmd_results.launch.py
ros2 launch c2hi exp3c_record_gesture_cmd_results.launch.py
```


# Analyzing the experimental results
```
cd ~/c2hi_ws/src/c2hi/scripts/analysis
mamba activate c2hi
python3 exp1_analyze_scene_results.py
python3 exp2_analyze_role_results.py
python3 exp3_analyze_cmd_results.py # Computes accuracy for different command modes
python3 make_audio_table_plot.py # Generates table and plot of verbal command mode
python3 make_summary_cmd_plot.py # Generates table and plot of best ar/verbal/gesture configs

```
