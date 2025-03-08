#!/usr/bin/env python

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Find best VAD, AM parameters
am_results_df = pd.read_csv('../../results/exp0_obs_models/speech_eval.csv')
am_results_df_sorted = am_results_df.sort_values(by='Command Error Rate',ascending=True)

# Get best AM params for each scene
scenes = ['campus','courtyard','lab','lobby','overall']
scenes_to_check = ['campus','courtyard','lab','lobby','overall']
am_results_dict = {}

best_models = {}

for _,row in am_results_df_sorted.iterrows():

    if row['Scene'] in scenes_to_check:
        scenes_to_check.remove(row['Scene'])

        best_models[row['Scene']] = {}
        best_models[row['Scene']] = row['Model']

        print(f"Best model for scene {row['Scene']} is {row['Model']} with error rate {row['Command Error Rate']}")
        
am_analysis_cols = ['Scene','Acoustic Model','Comment','cer', 'music', 'srp','tops' ]
am_analysis_df = pd.DataFrame(columns = am_analysis_cols)

for scene in scenes:
    for model_scene in scenes:
        model = best_models[model_scene]
        cer = am_results_df[((am_results_df['Model']==model) & (am_results_df['Scene']==scene))]['Command Error Rate'].values[0]
        temp_df = pd.DataFrame([[scene,model,f'best_{model_scene}',cer,0.,0.,0.]],columns = am_analysis_cols)
        am_analysis_df = pd.concat([am_analysis_df,temp_df],axis=0,ignore_index=True)


# Load command modality accuracy df
cmd_mode_acc_df = pd.read_csv('../../results/exp3_cmd_recognition/cmd_modality_accuracy.csv', index_col=0)

# Now, populate command modality accuracy 
for index,row in am_analysis_df.iterrows():
    for ssl_method in ['music','srp','tops']:

        am_analysis_df.at[index,ssl_method] = cmd_mode_acc_df.loc[f"config/exp3_cmd_cfgs/ar_role_verbal_{ssl_method}_{row['Comment']}_cmd.yaml"][f"{row['Scene']}_acc"]


# Generate plot data
model_dict = {'Best Overall (TL 7.0, LM 3.0, WS 3.0, SS 0.0)': 'best_overall',
              'Best Sidewalk (TL 7.0, LM 0.0, WS 3.0, SS 1.0)': 'best_campus',
              'Best Courtyard (TL 5.0, LM 1.0, WS 3.0, SS 0.0)': 'best_courtyard',
              'Best Lab (TL 7.0, LM -3.0, WS 0.0, SS -3.0)': 'best_lab',
              'Best Lobby (TL 3.0, LM 3.0, WS 3.0, SS 1.0)': 'best_lobby'
             }
              
scene_dict = {"Overall": 'overall',
              "Sidewalk": 'campus', 
              "Courtyard": 'courtyard',
              "Lab": 'lab',
              "Lobby": 'lobby'
             }
cer_data = {}
for model_key in model_dict.keys():
    model = model_dict[model_key]
    cer_data[model_key] = []

    for scene_key in scene_dict.keys():
        scene = scene_dict[scene_key]
        
        scene_acc = am_analysis_df[(am_analysis_df['Scene']==scene) & (am_analysis_df['Comment']==model)]['cer'].values[0]
        # print(scene_acc)
        cer_data[model_key].append(scene_acc)

music_data = {}
for model_key in model_dict.keys():
    model = model_dict[model_key]
    music_data[model_key] = []

    for scene_key in scene_dict.keys():
        scene = scene_dict[scene_key]
        
        scene_acc = am_analysis_df[(am_analysis_df['Scene']==scene) & (am_analysis_df['Comment']==model)]['music'].values[0]
        # print(scene_acc)
        music_data[model_key].append(scene_acc)

srp_data = {}
for model_key in model_dict.keys():
    model = model_dict[model_key]
    srp_data[model_key] = []

    for scene_key in scene_dict.keys():
        scene = scene_dict[scene_key]
        
        scene_acc = am_analysis_df[(am_analysis_df['Scene']==scene) & (am_analysis_df['Comment']==model)]['srp'].values[0]
        # print(scene_acc)
        srp_data[model_key].append(scene_acc)

tops_data = {}
for model_key in model_dict.keys():
    model = model_dict[model_key]
    tops_data[model_key] = []

    for scene_key in scene_dict.keys():
        scene = scene_dict[scene_key]
        
        scene_acc = am_analysis_df[(am_analysis_df['Scene']==scene) & (am_analysis_df['Comment']==model)]['tops'].values[0]
        # print(scene_acc)
        tops_data[model_key].append(scene_acc)


# Generate plots

x = np.arange(len(scene_dict.keys()))  # the label locations
width = 0.15  # the width of the bars
multiplier = -1

fig, axs = plt.subplots(1,1,layout='constrained',figsize=(6,3.5))

for attribute, measurement in cer_data.items():
    offset = width * multiplier
    rects = axs.bar(x + offset, measurement, width, label=attribute)
    axs.bar_label(rects, padding=0, fmt='%.2f')
    multiplier += 1

# Add some text for labels, title and custom x-axis tick labels, etc.
axs.set_ylabel('Command Error Rate')
axs.set_title('Best Audio Parameters by Scene: Ground Truth Localization')
axs.set_xticks(x + width, scene_dict.keys())
axs.legend(loc='upper left')
axs.set_ylim(0, 1.4)

fig.savefig('../../results/exp3_cmd_recognition/verbal_cer_plot.png',dpi=fig.dpi)

fig, axs = plt.subplots(3,1,layout='constrained',figsize=(6,6))
multiplier = -1
for attribute, measurement in music_data.items():
    offset = width * multiplier
    rects = axs[0].bar(x + offset, measurement, width, label=attribute)
    axs[0].bar_label(rects, padding=0, fmt='%.2f')
    multiplier += 1

# Add some text for labels, title and custom x-axis tick labels, etc.
# axs[0].set_ylabel('Command Recognition Accuracy')
axs[0].set_title('Best Audio Parameters by Scene: MUSIC SSL')
axs[0].set_xticks(x + width, scene_dict.keys())
axs[0].set_ylim(0, 0.5)

multiplier = -1
for attribute, measurement in srp_data.items():
    offset = width * multiplier
    rects = axs[1].bar(x + offset, measurement, width, label=attribute)
    axs[1].bar_label(rects, padding=0, fmt='%.2f')
    multiplier += 1

# Add some text for labels, title and custom x-axis tick labels, etc.
axs[1].set_ylabel('Command Recognition Accuracy')
axs[1].set_title('Best Audio Parameters by Scene: SRP-PHAT SSL')
axs[1].set_xticks(x + width, scene_dict.keys())
axs[1].set_ylim(0, 0.5)

multiplier = -1
for attribute, measurement in tops_data.items():
    offset = width * multiplier
    rects = axs[2].bar(x + offset, measurement, width, label=attribute)
    axs[2].bar_label(rects, padding=0, fmt='%.2f')
    multiplier += 1

# Add some text for labels, title and custom x-axis tick labels, etc.
# axs[2].set_ylabel('Command Recognition Accuracy')
axs[2].set_title('Best Audio Parameters by Scene: TOPS SSL')
axs[2].set_xticks(x + width, scene_dict.keys())
axs[2].set_ylim(0, 0.5)

fig.savefig('../../results/exp3_cmd_recognition/verbal_acc_plot.png',dpi=fig.dpi)
