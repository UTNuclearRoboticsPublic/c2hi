import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


vis_obs_columns = ['Vision Backbone','Role Description','Accuracy (campus)','Accuracy (courtyard)','Accuracy (lab)','Accuracy (lobby)','Accuracy (overall)']
vision_dict = {'RN50': 'ResNet50',
               'ViT-B32': 'ViT-B/32',
               'ViT-L14': 'ViT-L/14',
               'basic_description': 'Basic',
               'detailed_description': 'Detailed'}

vis_role_results_df = pd.DataFrame(columns=vis_obs_columns)

# Initialize blank dataframe
for model_key in ['RN50','ViT-B32','ViT-L14']:
    for desc_key in ['basic_description','detailed_description']:
        analysis_df = pd.DataFrame([[vision_dict[model_key],vision_dict[desc_key],0.,0.,0.,0.,0.]],columns=vis_obs_columns)
        vis_role_results_df = pd.concat((vis_role_results_df,analysis_df),ignore_index=True)

scene_offset = 2 # 2 columns in
for scene_idx,scene in enumerate(['campus','courtyard','lab','lobby','overall']):
    row_idx=0
    for model_key in ['RN50','ViT-B32','ViT-L14']:
        for desc_key in ['basic_description','detailed_description']:
            
            # Compute accuracy
            visual_obs_model_df = pd.read_csv(f"../../results/exp0_obs_models/role_recognition/{scene}/{model_key}_{desc_key}_role_obs_model_raw.csv",index_col=0)
    
            true_pos = 0
            for ii in range(visual_obs_model_df.to_numpy().shape[0]):
                true_pos += visual_obs_model_df.to_numpy()[ii,ii]
    
            total_est = visual_obs_model_df.to_numpy().sum()
            acc = true_pos/total_est

            vis_role_results_df.iat[row_idx,scene_idx+scene_offset] = acc

            row_idx +=1


vis_role_results_df.to_csv('../../results/exp0_obs_models/role_recognition/role_rec_obs_model_accuracy.csv')