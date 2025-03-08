#!/usr/bin/env python3

import re
import pandas as pd
import numpy as np

# Helper functions
def count_recognized_commands(cmd_rec_mask):
    count=0
    last_state = False
    
    for cmd_rec in cmd_rec_mask:
        if ((cmd_rec == False) & (last_state == True)):
            count+=1

        last_state = cmd_rec

    return count

# Constants
results_filepaths = ['../../results/exp3_cmd_recognition/ar_tag/hierarchical_cmd_results.csv',
                     '../../results/exp3_cmd_recognition/verbal/hierarchical_cmd_results.csv']

# Initialize output
cols = []
for scene in ['campus','courtyard','lab','lobby','overall']:
    cols.append(f"{scene}_correct")
    cols.append(f"{scene}_total")
    cols.append(f"{scene}_acc")

full_results_df = pd.DataFrame(columns=cols)

# Iterate through different configurations
for filepath in results_filepaths:  

    # Load results data
    hierarchical_cmd_results_df = pd.read_csv(filepath,index_col=0)
    hierarchical_cmd_results_df = hierarchical_cmd_results_df.dropna()
    print(hierarchical_cmd_results_df)


    # Parse configuration names and determine which command modalities to evaluate for accuracy
    config_metadata = {}
    for config in hierarchical_cmd_results_df['config'].unique():
        config_metadata[config] = {}

        print(config)

        cmd_mode_re = re.match(".+_role_(.+)_cmd\.yaml",config).group(1)
        print(cmd_mode_re)

        config_metadata[config]['cmd_modes'] = []

        if 'ar' in cmd_mode_re:
            config_metadata[config]['cmd_modes'].append('artag')

        if 'verbal' in cmd_mode_re:
            config_metadata[config]['cmd_modes'].append('verbal')

    print(config_metadata)


    results_df = pd.DataFrame(np.zeros((len(config_metadata.keys()),len(cols))),index=config_metadata.keys(),columns=cols)

    for cfg in config_metadata.keys():
        for cmd_modality in config_metadata[cfg]['cmd_modes']:
            for scene in ['campus','courtyard','lab','lobby']:
                for role in ['pedestrian','supervisor','teammate']:
                    for actual_cmd in ['advance','attention','follow-me','halt','move-forward','move-in-reverse','rally']:

                        print(cfg)
                        print(cmd_modality)
                        print(scene)
                        print(role)
                        print(actual_cmd)

                        cmd_series_mask = ((hierarchical_cmd_results_df['config']==cfg) &
                                        (hierarchical_cmd_results_df['scene']==scene) & 
                                        (hierarchical_cmd_results_df['cmd']==actual_cmd) & 
                                        (hierarchical_cmd_results_df['role']==role) & 
                                        (hierarchical_cmd_results_df['cmd_mode']==cmd_modality))
                
                        command_series_df = hierarchical_cmd_results_df[cmd_series_mask]

                        cmd_correct_mask = command_series_df['estimated command'] == actual_cmd

                        commands_expected = round(len(command_series_df)/6)
                        results_df.loc[cfg,f"{scene}_total"] += commands_expected
                        results_df.loc[cfg,"overall_total"] += commands_expected
                        
                        commands_correct = count_recognized_commands(cmd_correct_mask)
                        results_df.loc[cfg,f"{scene}_correct"] += commands_correct
                        results_df.loc[cfg,"overall_correct"] += commands_correct

                        print("Expected: " + str(commands_expected))
                        print("Correct: " + str(commands_correct))
                        print()

        for scene in ['campus','courtyard','lab','lobby', 'overall']:
            results_df.loc[cfg,f"{scene}_acc"] = results_df.loc[cfg,f"{scene}_correct"]/results_df.loc[cfg,f"{scene}_total"]

    full_results_df = pd.concat([full_results_df,results_df],axis=0)


print(full_results_df)
full_results_df.to_csv('../../results/exp3_cmd_recognition/cmd_modality_accuracy.csv')