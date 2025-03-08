import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


gesture_obs_columns = ['Gesture Time Window','Training Dataset','Eval Dataset','Accuracy']
gesture_name_map = {
    'real': 'Real',
    'syn': 'Synthetic',
    'real_100': 'Real - Full',
    'real_80': 'Real - Training',
    'real_20': 'Real - Validation',
    'syn_100': 'Synthetic - Full',
    'syn_80': 'Synthetic - Training',
    'syn_20': 'Synthetic - Validation',
    '8': '0.5 seconds',
    '16': '1.0 seconds',
    '24': '1.5 seconds'
}

gesture_obs_results_df = pd.DataFrame(columns=gesture_obs_columns)

# Initialize blank dataframe
for window_len in ['8','16','24']:
    for training_data in ['real_100','real_80','syn_100','syn_80']:
        for eval_data in ['real_100','real_20','syn_20']:

            print(f"Model: {training_data}_{window_len}: Eval {gesture_name_map[eval_data]}")
           
            # Compute accuracy
            obs_model_df = pd.read_csv(f"../../../gesture_recognition/gesture_recognition_ros2/models/{training_data}_withnone_wl{window_len}/{eval_data}_{window_len}_conf.csv",index_col=0)
    
            true_pos = 0
            for ii in range(obs_model_df.to_numpy().shape[0]):
                true_pos += obs_model_df.to_numpy()[ii,ii]
    
            total_est = obs_model_df.to_numpy().sum()
            acc = true_pos/total_est

            print(acc)
            print()

            analysis_df = pd.DataFrame([[gesture_name_map[window_len],gesture_name_map[training_data],gesture_name_map[eval_data],acc]],columns=gesture_obs_columns)
            gesture_obs_results_df = pd.concat((gesture_obs_results_df,analysis_df),ignore_index=True)


gesture_obs_results_df.to_csv('../../results/exp0_obs_models/cmd_recognition/gesture_rec_model_accuracy.csv')