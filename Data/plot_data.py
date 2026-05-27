import os
import pandas as pd
import matplotlib.pyplot as plt
import yaml
import argparse

# Configure matplotlib to use LaTeX-like fonts
plt.rcParams.update({
    "font.family": "serif",
    "mathtext.fontset": "cm",
    "font.serif": ["Computer Modern Roman", "Times New Roman", "DejaVu Serif"],
    "axes.formatter.use_mathtext": True
})

def create_default_config(config_path):
    default_config = {
        'start_time': 0.0,
        'finish_time': -1.0  # -1.0 means plot until the end
    }
    with open(config_path, 'w') as f:
        yaml.dump(default_config, f, default_flow_style=False, sort_keys=False)
    return default_config

def plot_csv(csv_path, config):
    print(f"Reading {csv_path} ...")
    df = pd.read_csv(csv_path)
    
    start_t = config.get('start_time', 0.0)
    finish_t = config.get('finish_time', -1.0)
    
    # Filter data based on config
    df_filtered = df[df['timestamp'] >= start_t]
    if finish_t >= 0:
        df_filtered = df_filtered[df_filtered['timestamp'] <= finish_t]
        
    if df_filtered.empty:
        print(f"Warning: No data left in {csv_path} after applying time filter.")
        return

    # Create 4x2 subplot
    fig, axs = plt.subplots(4, 2, figsize=(12, 16))
    fig.suptitle(f"Data Plot: {os.path.basename(os.path.dirname(csv_path))}", fontsize=16)

    t = df_filtered['timestamp'] - start_t

    # --- Motor 1 ---
    # Position
    axs[0, 0].plot(t, df_filtered['ref_q_1'], label=r'$q_{ref}$', linestyle='--', color='red')
    axs[0, 0].plot(t, df_filtered['state_q_1'], label=r'$q_{state}$', color='blue')
    axs[0, 0].set_title(r'Motor 1 Position ($q$)')
    axs[0, 0].set_ylabel('Position [rad]')
    axs[0, 0].legend()
    axs[0, 0].grid(True, linestyle=':', alpha=0.7)

    # Torque
    axs[1, 0].plot(t, df_filtered['cmd_tau_1'], label=r'$\tau_{cmd}$', linestyle='--', color='red')
    axs[1, 0].plot(t, df_filtered['state_tau_est_1'], label=r'$\tau_{est}$', color='green')
    axs[1, 0].set_title(r'Motor 1 Torque ($\tau$)')
    axs[1, 0].set_ylabel('Torque [Nm]')
    axs[1, 0].legend()
    axs[1, 0].grid(True, linestyle=':', alpha=0.7)

    # Position Error
    pos_error_1 = df_filtered['ref_q_1'] - df_filtered['state_q_1']
    axs[2, 0].plot(t, pos_error_1, color='purple')
    axs[2, 0].set_title(r'Motor 1 Position Error ($q_{ref} - q_{state}$)')
    axs[2, 0].set_ylabel('Error [rad]')
    axs[2, 0].grid(True, linestyle=':', alpha=0.7)

    # Torque Error
    tau_error_1 = df_filtered['cmd_tau_1'] - df_filtered['state_tau_est_1']
    axs[3, 0].plot(t, tau_error_1, color='orange')
    axs[3, 0].set_title(r'Motor 1 Torque Error ($\tau_{cmd} - \tau_{est}$)')
    axs[3, 0].set_xlabel('Time [s]')
    axs[3, 0].set_ylabel('Error [Nm]')
    axs[3, 0].grid(True, linestyle=':', alpha=0.7)

    # --- Motor 2 ---
    # Position
    axs[0, 1].plot(t, df_filtered['ref_q_2'], label=r'$q_{ref}$', linestyle='--', color='red')
    axs[0, 1].plot(t, df_filtered['state_q_2'], label=r'$q_{state}$', color='blue')
    axs[0, 1].set_title(r'Motor 2 Position ($q$)')
    axs[0, 1].set_ylabel('Position [rad]')
    axs[0, 1].legend()
    axs[0, 1].grid(True, linestyle=':', alpha=0.7)

    # Torque
    axs[1, 1].plot(t, df_filtered['cmd_tau_2'], label=r'$\tau_{cmd}$', linestyle='--', color='red')
    axs[1, 1].plot(t, df_filtered['state_tau_est_2'], label=r'$\tau_{est}$', color='green')
    axs[1, 1].set_title(r'Motor 2 Torque ($\tau$)')
    axs[1, 1].set_ylabel('Torque [Nm]')
    axs[1, 1].legend()
    axs[1, 1].grid(True, linestyle=':', alpha=0.7)

    # Position Error
    pos_error_2 = df_filtered['ref_q_2'] - df_filtered['state_q_2']
    axs[2, 1].plot(t, pos_error_2, color='purple')
    axs[2, 1].set_title(r'Motor 2 Position Error ($q_{ref} - q_{state}$)')
    axs[2, 1].set_ylabel('Error [rad]')
    axs[2, 1].grid(True, linestyle=':', alpha=0.7)

    # Torque Error
    tau_error_2 = df_filtered['cmd_tau_2'] - df_filtered['state_tau_est_2']
    axs[3, 1].plot(t, tau_error_2, color='orange')
    axs[3, 1].set_title(r'Motor 2 Torque Error ($\tau_{cmd} - \tau_{est}$)')
    axs[3, 1].set_xlabel('Time [s]')
    axs[3, 1].set_ylabel('Error [Nm]')
    axs[3, 1].grid(True, linestyle=':', alpha=0.7)

    plt.tight_layout()
    
    # Save to PDF
    pdf_path = os.path.splitext(csv_path)[0] + '_plot.pdf'
    plt.savefig(pdf_path, format='pdf', bbox_inches='tight')
    plt.close()
    print(f"Saved plot to {pdf_path}")

def main():
    parser = argparse.ArgumentParser(description="Plot extracted ROS 2 bag data")
    parser.add_argument('--overwrite', '-o', action='store_true', help="Overwrite existing PDF plots")
    args = parser.parse_args()

    base_dir = '/home/felipe/go2_simulation/Data'
    
    # Find all extracted_data.csv files
    for root, dirs, files in os.walk(base_dir):
        if 'extracted_data.csv' in files:
            csv_path = os.path.join(root, 'extracted_data.csv')
            pdf_path = os.path.splitext(csv_path)[0] + '_plot.pdf'
            
            if os.path.exists(pdf_path) and not args.overwrite:
                print(f"Skipping {csv_path}, plot already exists. Use --overwrite to regenerate.")
                continue
                
            config_path = os.path.join(root, 'plot_config.yaml')
            
            # Read or create config
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
            else:
                print(f"Creating default config for {csv_path}")
                config = create_default_config(config_path)
                
            plot_csv(csv_path, config)

if __name__ == '__main__':
    main()
