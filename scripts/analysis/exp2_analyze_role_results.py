 #!/usr/bin/env python3

import os
import re
import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


# Load result data
filepath = '../../results/exp2_role_recognition/hierarchical_cmd_results.csv'
hierarchical_cmd_results_df = pd.read_csv(filepath,index_col=0)
hierarchical_cmd_results_df = hierarchical_cmd_results_df.dropna()

max_num_updates = 37

role_rec_results = {}

config_dict = {
    'Fiducial': 'ar_role',
    'Visual (Best Overall)': 'best_overall_vis_role'
}

# Initialize hashmap to store results
for config in config_dict.keys():
    role_rec_results[config] = {}
    
    for scene in ['overall','campus','courtyard','lab','lobby']:
        role_rec_results[config][scene] = {}
        role_rec_results[config][scene]['true_pos_role_est'] = np.zeros((max_num_updates))
        role_rec_results[config][scene]['total_role_est'] = np.zeros((max_num_updates))

# Populate hashmap
for config in config_dict.keys():
    role_rec_config_str = config_dict[config]

    for scene in ['campus','courtyard','lab','lobby']:
        for role in ['supervisor','teammate','pedestrian']:
                for cmd_mode in ['artag','gest_verb','gesture','verbal']:
                    for cmd in ['move-in-reverse','move-forward','follow-me','rally','halt','advance','attention']:
    
                        # Get est roles from epoch
                        df_mask = ((hierarchical_cmd_results_df['scene']==scene) 
                                   & (hierarchical_cmd_results_df['role']==role)
                                   & (hierarchical_cmd_results_df['cmd_mode']==cmd_mode)
                                   & (hierarchical_cmd_results_df['config'].str.contains(role_rec_config_str))
                                   & (hierarchical_cmd_results_df['cmd']==cmd))
                        est_role_df = hierarchical_cmd_results_df[df_mask]

                        if len(est_role_df)>0:
                            true_pos_mask = (est_role_df['role'] == est_role_df['estimated role'])
                                   
                            true_pos_role_est= np.zeros((max_num_updates))
                            true_pos_role_est[:len(est_role_df)] = true_pos_mask
                            
                            total_role_est = np.zeros((max_num_updates))
                            total_role_est[:len(est_role_df)] = np.ones(len(est_role_df))
        
                            role_rec_results[config]['overall']['true_pos_role_est'] += true_pos_role_est
                            role_rec_results[config][scene]['true_pos_role_est'] += true_pos_role_est
                            
                            role_rec_results[config]['overall']['total_role_est'] += total_role_est
                            role_rec_results[config][scene]['total_role_est'] += total_role_est

# Generate plot
fig, axes = plt.subplots(1,len(config_dict.keys()), sharex=True, sharey=True, figsize=(8,4))

for idx, config in enumerate(config_dict.keys()):
    ax = axes[idx]
    ax.set_xlim((1,25))
    ax.set_ylim((.5,1.01))
    ax.set_title(f"{config}",fontsize=10)

    ax.plot(role_rec_results[config]['overall']['true_pos_role_est'] / role_rec_results[config]['overall']['total_role_est'],linestyle='-',label='overall')
    for scene in ['campus','courtyard','lab','lobby']:
        acc = role_rec_results[config][scene]['true_pos_role_est']/ role_rec_results[config][scene]['total_role_est']
        ax.plot(acc,linestyle='--',label=scene)
        
ax.legend(loc="lower right")

fig.suptitle('Role Classification Accuracy by Method')
fig.text(0.5, 0.02, 'Number of Bayesian Updates', ha='center', va='center')
fig.text(0.06, 0.5, 'Classification Accuracy', ha='center', va='center', rotation='vertical')

# Save figure and table data
fig.savefig('../../results/exp2_role_recognition/role_acc_plot.png',dpi=fig.dpi)
hierarchical_cmd_results_df.to_csv('../../results/exp2_role_recognition/role_acc_data.csv')