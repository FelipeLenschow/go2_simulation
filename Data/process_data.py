import os
import pandas as pd
import numpy as np

def extract_and_average_cycles(df):
    if df.empty:
        return df, 0.0

    valid_df = df.dropna(subset=['ref_q_1', 'state_q_1'])
    if valid_df.empty:
        return df, 0.0

    # Find downward crossings of 1.0
    crossings = valid_df.index[(valid_df['ref_q_1'].shift(1) > 1.0) & (valid_df['ref_q_1'] <= 1.0)]
    
    if len(crossings) == 0:
        return valid_df, 0.0

    first_cross_time = valid_df.loc[crossings[0], 'timestamp']
    
    # We will interpolate onto a common grid of 15 seconds: from 0.0s to 15.0s
    common_time_grid = np.arange(0.0, 15.0, 0.005) # 200 Hz
    
    cycle_dfs = []
    
    for cross_idx in crossings:
        cross_time = valid_df.loc[cross_idx, 'timestamp']
        
        # Extract cycle window (start 1.5s before crossing, end 13.5s after)
        start_time = cross_time - 1.5
        end_time = cross_time + 13.5
        
        cycle_data = valid_df[(valid_df['timestamp'] >= start_time) & (valid_df['timestamp'] < end_time)].copy()
        
        # Ensure strictly increasing time for interpolation
        cycle_data = cycle_data.drop_duplicates(subset=['timestamp'])
        
        if len(cycle_data) < 100:
            continue
            
        # Shift time so that start_time maps to 0.0 (crossing will be at t=1.5s)
        relative_time = cycle_data['timestamp'].values - start_time
        
        # Interpolate variables
        interpolated = {'aligned_time': common_time_grid}
        for col in cycle_data.columns:
            if col not in ['timestamp', 'aligned_time']:
                interpolated[col] = np.interp(common_time_grid, relative_time, cycle_data[col].values)
                
        cycle_dfs.append(pd.DataFrame(interpolated))
        
    if not cycle_dfs:
        return df, 0.0
        
    # Average all cycles
    all_cycles = pd.concat(cycle_dfs)
    averaged_df = all_cycles.groupby('aligned_time').mean().reset_index()
    
    return averaged_df, first_cross_time

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    for domain in ['Sim', 'Real']:
        for mode in [1, 2, 3, 4]:
            folder_path = os.path.join(base_dir, domain, f'Control_mode_{mode}')
            csv_path = os.path.join(folder_path, 'extracted_data.csv')
            
            if os.path.exists(csv_path):
                print(f"Processing {domain} Mode {mode}...")
                df = pd.read_csv(csv_path)
                averaged_df, start_time = extract_and_average_cycles(df)
                
                # Save processed data
                processed_path = os.path.join(folder_path, 'processed_data.csv')
                averaged_df.to_csv(processed_path, index=False)
                
                # Write config.yaml
                config_path = os.path.join(folder_path, 'config.yaml')
                with open(config_path, 'w') as f:
                    f.write(f"start_time: {start_time:.4f}\n")

if __name__ == '__main__':
    main()
