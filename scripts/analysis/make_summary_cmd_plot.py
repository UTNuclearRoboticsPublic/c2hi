#!/usr/bin/env python

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load command modality accuracy df
cmd_mode_acc_df = pd.read_csv('../../results/exp3_cmd_recognition/cmd_modality_accuracy.csv', index_col=0)

# Generate plot data
model_dict = {'Best Fiducial': 'config/exp3_cmd_cfgs/ar_role_ar_cmd.yaml',
              'Best Verbal': 'config/exp3_cmd_cfgs/ar_role_verbal_srp_best_lobby_cmd.yaml',
              'Best Gesture': 'config/exp3_cmd_cfgs/ar_role_gesture_r80_w16_cmd.yaml'
             }
              
scene_dict = {"Overall": 'overall_acc',
              "Sidewalk": 'campus_acc', 
              "Courtyard": 'courtyard_acc',
              "Lab": 'lab_acc',
              "Lobby": 'lobby_acc'
             }

acc_data = {}
for model_key in model_dict.keys():
    model = model_dict[model_key]
    acc_data[model_key] = []

    for scene_key in scene_dict.keys():
        scene = scene_dict[scene_key]
        
        cmd_acc = cmd_mode_acc_df.at[model,scene]
        acc_data[model_key].append(cmd_acc)

print(acc_data)

# Generate plot

x = np.arange(len(scene_dict.keys()))  # the label locations
width = 0.15  # the width of the bars
multiplier = 0

fig, ax = plt.subplots(1,1,layout='constrained',figsize=(6,3.5))

for attribute, measurement in acc_data.items():
    offset = width * multiplier
    rects = ax.bar(x + offset, measurement, width, label=attribute)
    ax.bar_label(rects, padding=0, fmt='%.2f')
    multiplier += 1

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Command Recognition Accuracy')
ax.set_title('Summary of Command Method Accuracy by Scene')
ax.set_xticks(x + width, scene_dict.keys())
ax.legend(loc='best')
ax.set_ylim(0., 1.)

fig.savefig('../../results/exp3_cmd_recognition/overall_acc_plot.png',dpi=fig.dpi)
