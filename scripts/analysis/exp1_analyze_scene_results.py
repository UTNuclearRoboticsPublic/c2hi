#!/usr/bin/env python3

import os
import re
import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

max_n_updates = 30

scene_recognition_filepath = '../../results/exp1a_scene_recognition'

vision_dict = {'rn50': 'ResNet50',
               'vitb32': 'ViT-B/32',
               'vitl14': 'ViT-L/14',
               'basic': 'Basic',
               'detailed': 'Detailed'}

results_columns = ['Vision Backbone','Scene Description','Method','Number of Updates','Accuracy']
scene_recognition_results_df = pd.DataFrame(columns = results_columns)

# Compute results
for _, _, cfg_files in os.walk(scene_recognition_filepath):
    for cfg_file in cfg_files:

        # Get detailed names for results dataframe
        x = re.search('scene_results_(.+?)_(.+?).csv',cfg_file)

        if x is None:
            continue

        vision_backbone_key = x.group(1)
        vision_description_key = x.group(2)
        vision_backbone = vision_dict[vision_backbone_key]
        vision_description = vision_dict[vision_description_key]

        # Load the experiment results
        results_df = pd.read_csv(os.path.join(scene_recognition_filepath, cfg_file),index_col=0)

        for ii in range(1,max_n_updates):
            # Compute scene recognition accuracy of different modalities
            audio_results_df = results_df[(results_df['scene_estimation_mode']=='audio') & (results_df['n_updates']==ii)]
            audio_true_pos_df= results_df[(results_df['scene_estimation_mode']=='audio') & (results_df['n_updates']==ii) & (results_df['scene_est']==results_df['scene'])]
            clip_results_df = results_df[(results_df['scene_estimation_mode']=='clip') & (results_df['n_updates']==ii)]
            clip_true_pos_df = results_df[(results_df['scene_estimation_mode']=='clip') & (results_df['n_updates']==ii) & (results_df['scene_est']==results_df['scene'])]
            joint_results_df = results_df[(results_df['scene_estimation_mode']=='joint') & (results_df['n_updates']==ii) ]
            joint_true_pos_df = results_df[(results_df['scene_estimation_mode']=='joint') & (results_df['n_updates']==ii) & (results_df['scene_est']==results_df['scene'])]
            audio_acc = float(audio_true_pos_df.shape[0]/audio_results_df.shape[0])
            vis_acc = float(clip_true_pos_df.shape[0]/clip_results_df.shape[0])        
            joint_acc = float(joint_true_pos_df.shape[0]/joint_results_df.shape[0])

            # Add results to DF
            config_results_df = pd.DataFrame([[vision_backbone, vision_description, 'audio', ii, audio_acc]],columns = results_columns)
            scene_recognition_results_df = pd.concat((scene_recognition_results_df,config_results_df), ignore_index=True)
            config_results_df = pd.DataFrame([[vision_backbone, vision_description, 'visual', ii, vis_acc]],columns = results_columns)
            scene_recognition_results_df = pd.concat((scene_recognition_results_df,config_results_df), ignore_index=True)
            config_results_df = pd.DataFrame([[vision_backbone, vision_description, 'joint', ii, joint_acc]],columns = results_columns)
            scene_recognition_results_df = pd.concat((scene_recognition_results_df,config_results_df), ignore_index=True)

# Generate plots
fig, axes = plt.subplots(3,2, sharex=True, sharey=True, figsize=(5,7))

for col_idx, desc in enumerate(['Basic','Detailed']):
    for row_idx,model in enumerate(['ResNet50','ViT-B/32','ViT-L/14']):
        ax = axes[row_idx, col_idx]
        ax.set_xlim((1,25))
        ax.set_ylim((.5,1.))
        ax.set_title(f"{model}-{desc}",fontsize=10)


        audio_mask = ((scene_recognition_results_df['Vision Backbone'] == model) &
                      (scene_recognition_results_df['Scene Description'] == desc) & 
                     (scene_recognition_results_df['Method'] == 'audio'))
        vision_mask = ((scene_recognition_results_df['Vision Backbone'] == model) &
                      (scene_recognition_results_df['Scene Description'] == desc) & 
                     (scene_recognition_results_df['Method'] == 'visual'))
        joint_mask = ((scene_recognition_results_df['Vision Backbone'] == model) &
                      (scene_recognition_results_df['Scene Description'] == desc) & 
                     (scene_recognition_results_df['Method'] == 'joint'))
        
        audio_data = scene_recognition_results_df[audio_mask]
        vision_data = scene_recognition_results_df[vision_mask]
        joint_data = scene_recognition_results_df[joint_mask]

        ax.plot(audio_data['Number of Updates'],audio_data['Accuracy'],linestyle='--',label='audio')
        ax.plot(vision_data['Number of Updates'],vision_data['Accuracy'],linestyle='--',label='vision')
        ax.plot(joint_data['Number of Updates'],joint_data['Accuracy'],label='1:1 joint')
ax.legend(loc="lower right")

fig.suptitle('Scene Classification Accuracy for Various Visual Encoders')
fig.text(0.5, 0.02, 'Number of Bayesian Updates', ha='center', va='center')
fig.text(0.02, 0.5, 'Classification Accuracy', ha='center', va='center', rotation='vertical')

# Save figure and table data
fig.savefig('../../results/exp1a_scene_recognition/scene_acc_plot.png',dpi=fig.dpi)
scene_recognition_results_df.to_csv('../../results/exp1a_scene_recognition/scene_acc_data.csv')
