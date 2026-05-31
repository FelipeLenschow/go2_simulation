import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def calc_rms(series):
    return np.sqrt(np.mean(series**2))

# Configure matplotlib
plt.rcParams.update({
    "font.family": "serif",
    "mathtext.fontset": "cm",
    "font.serif": ["Computer Modern Roman", "Times New Roman", "DejaVu Serif"],
    "axes.formatter.use_mathtext": True
})

def load_processed_data(base_dir):
    data = {}
    for domain in ['Sim', 'Real']:
        data[domain] = {}
        for mode in [1, 2, 3, 4]:
            csv_path = os.path.join(base_dir, domain, f'Control_mode_{mode}', 'processed_data.csv')
            if os.path.exists(csv_path):
                data[domain][mode] = pd.read_csv(csv_path)
            else:
                data[domain][mode] = None
    return data

def plot_sim_vs_real(data, out_dir):
    # For each mode, plot Sim vs Real
    for mode in [1, 2, 3, 4]:
        sim_df = data['Sim'][mode]
        real_df = data['Real'][mode]
        
        if sim_df is None or real_df is None:
            continue
            
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f"Sim vs Real - Control Mode {mode}", fontsize=16)
        
        # M1 Position Error
        err_sim_1 = sim_df['ref_q_1'] - sim_df['state_q_1']
        err_real_1 = real_df['ref_q_1'] - real_df['state_q_1']
        rms_err_sim_1 = calc_rms(err_sim_1)
        rms_err_real_1 = calc_rms(err_real_1)
        axs[0, 0].plot(sim_df['aligned_time'], err_sim_1, label=f'Sim (RMS: {rms_err_sim_1:.4f})', color='blue')
        axs[0, 0].plot(real_df['aligned_time'], err_real_1, label=f'Real (RMS: {rms_err_real_1:.4f})', color='red', linestyle='--')
        axs[0, 0].set_title('Motor 1 Position Error')
        axs[0, 0].set_ylabel('Error [rad]')
        axs[0, 0].legend()
        axs[0, 0].grid(True)
        
        # M1 Torque
        rms_tau_sim_1 = calc_rms(sim_df['state_tau_est_1'])
        rms_tau_real_1 = calc_rms(real_df['state_tau_est_1'])
        axs[1, 0].plot(sim_df['aligned_time'], sim_df['state_tau_est_1'], label=f'Sim (RMS: {rms_tau_sim_1:.2f})', color='blue')
        axs[1, 0].plot(real_df['aligned_time'], real_df['state_tau_est_1'], label=f'Real (RMS: {rms_tau_real_1:.2f})', color='red', linestyle='--')
        axs[1, 0].set_title('Motor 1 Torque')
        axs[1, 0].set_ylabel('Torque [Nm]')
        axs[1, 0].set_xlabel('Time [s]')
        axs[1, 0].legend()
        axs[1, 0].grid(True)
        
        # M2 Position Error
        err_sim_2 = sim_df['ref_q_2'] - sim_df['state_q_2']
        err_real_2 = real_df['ref_q_2'] - real_df['state_q_2']
        rms_err_sim_2 = calc_rms(err_sim_2)
        rms_err_real_2 = calc_rms(err_real_2)
        axs[0, 1].plot(sim_df['aligned_time'], err_sim_2, label=f'Sim (RMS: {rms_err_sim_2:.4f})', color='blue')
        axs[0, 1].plot(real_df['aligned_time'], err_real_2, label=f'Real (RMS: {rms_err_real_2:.4f})', color='red', linestyle='--')
        axs[0, 1].set_title('Motor 2 Position Error')
        axs[0, 1].set_ylabel('Error [rad]')
        axs[0, 1].legend()
        axs[0, 1].grid(True)
        
        # M2 Torque
        rms_tau_sim_2 = calc_rms(sim_df['state_tau_est_2'])
        rms_tau_real_2 = calc_rms(real_df['state_tau_est_2'])
        axs[1, 1].plot(sim_df['aligned_time'], sim_df['state_tau_est_2'], label=f'Sim (RMS: {rms_tau_sim_2:.2f})', color='blue')
        axs[1, 1].plot(real_df['aligned_time'], real_df['state_tau_est_2'], label=f'Real (RMS: {rms_tau_real_2:.2f})', color='red', linestyle='--')
        axs[1, 1].set_title('Motor 2 Torque')
        axs[1, 1].set_ylabel('Torque [Nm]')
        axs[1, 1].set_xlabel('Time [s]')
        axs[1, 1].legend()
        axs[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, f'Sim_vs_Real_Mode_{mode}.pdf'))
        plt.close()

def plot_mode_comparison(data, out_dir, domain, mode_a, mode_b, label_a, label_b):
    df_a = data[domain].get(mode_a)
    df_b = data[domain].get(mode_b)
    
    if df_a is None or df_b is None:
        return
        
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f"{domain} Comparison: {label_a} vs {label_b}", fontsize=16)
    
    # M1 Position Error
    err_a_1 = df_a['ref_q_1'] - df_a['state_q_1']
    err_b_1 = df_b['ref_q_1'] - df_b['state_q_1']
    rms_err_a_1 = calc_rms(err_a_1)
    rms_err_b_1 = calc_rms(err_b_1)
    axs[0, 0].plot(df_a['aligned_time'], err_a_1, label=f'{label_a} (RMS: {rms_err_a_1:.4f})', color='blue')
    axs[0, 0].plot(df_b['aligned_time'], err_b_1, label=f'{label_b} (RMS: {rms_err_b_1:.4f})', color='red', linestyle='--')
    axs[0, 0].set_title('Motor 1 Position Error')
    axs[0, 0].set_ylabel('Error [rad]')
    axs[0, 0].legend()
    axs[0, 0].grid(True)
    
    # M1 Torque
    rms_tau_a_1 = calc_rms(df_a['state_tau_est_1'])
    rms_tau_b_1 = calc_rms(df_b['state_tau_est_1'])
    axs[1, 0].plot(df_a['aligned_time'], df_a['state_tau_est_1'], label=f'{label_a} (RMS: {rms_tau_a_1:.2f})', color='blue')
    axs[1, 0].plot(df_b['aligned_time'], df_b['state_tau_est_1'], label=f'{label_b} (RMS: {rms_tau_b_1:.2f})', color='red', linestyle='--')
    axs[1, 0].set_title('Motor 1 Torque')
    axs[1, 0].set_ylabel('Torque [Nm]')
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].legend()
    axs[1, 0].grid(True)
    
    # M2 Position Error
    err_a_2 = df_a['ref_q_2'] - df_a['state_q_2']
    err_b_2 = df_b['ref_q_2'] - df_b['state_q_2']
    rms_err_a_2 = calc_rms(err_a_2)
    rms_err_b_2 = calc_rms(err_b_2)
    axs[0, 1].plot(df_a['aligned_time'], err_a_2, label=f'{label_a} (RMS: {rms_err_a_2:.4f})', color='blue')
    axs[0, 1].plot(df_b['aligned_time'], err_b_2, label=f'{label_b} (RMS: {rms_err_b_2:.4f})', color='red', linestyle='--')
    axs[0, 1].set_title('Motor 2 Position Error')
    axs[0, 1].set_ylabel('Error [rad]')
    axs[0, 1].legend()
    axs[0, 1].grid(True)
    
    # M2 Torque
    rms_tau_a_2 = calc_rms(df_a['state_tau_est_2'])
    rms_tau_b_2 = calc_rms(df_b['state_tau_est_2'])
    axs[1, 1].plot(df_a['aligned_time'], df_a['state_tau_est_2'], label=f'{label_a} (RMS: {rms_tau_a_2:.2f})', color='blue')
    axs[1, 1].plot(df_b['aligned_time'], df_b['state_tau_est_2'], label=f'{label_b} (RMS: {rms_tau_b_2:.2f})', color='red', linestyle='--')
    axs[1, 1].set_title('Motor 2 Torque')
    axs[1, 1].set_ylabel('Torque [Nm]')
    axs[1, 1].set_xlabel('Time [s]')
    axs[1, 1].legend()
    axs[1, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, f'{domain}_Comparison_Mode_{mode_a}_vs_{mode_b}.pdf'))
    plt.close()

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    data = load_processed_data(base_dir)
    
    out_dir = os.path.join(base_dir, 'plots')
    os.makedirs(out_dir, exist_ok=True)
    
    # 1. Sim vs Real plots
    plot_sim_vs_real(data, out_dir)
    
    # 2. Compare 1 vs 2 (PD vs PDG) and 3 vs 4 (PID vs PIDG)
    for domain in ['Sim', 'Real']:
        plot_mode_comparison(data, out_dir, domain, 1, 2, 'Mode 1 (PD)', 'Mode 2 (PDG)')
        plot_mode_comparison(data, out_dir, domain, 3, 4, 'Mode 3 (PID)', 'Mode 4 (PIDG)')

if __name__ == '__main__':
    main()
