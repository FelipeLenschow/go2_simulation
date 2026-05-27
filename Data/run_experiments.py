#!/usr/bin/env python3
import subprocess
import time
import os
import signal
import re
import datetime

workspace_dir = "/home/felipe/go2_simulation"
yaml_path = os.path.join(workspace_dir, "go2_description/config/go2_sim.yaml")
control_modes = [1, 2, 3, 4]

# Function to run a command with sourced environment
def run_sourced(cmd, bg=False):
    full_cmd = f"source install/setup.bash && {cmd}"
    if bg:
        # We use preexec_fn=os.setsid to create a new process group.
        # This allows us to send a signal to the entire group later (e.g. for graceful shutdown).
        return subprocess.Popen(full_cmd, shell=True, cwd=workspace_dir, executable='/bin/bash', preexec_fn=os.setsid)
    else:
        return subprocess.run(full_cmd, shell=True, cwd=workspace_dir, executable='/bin/bash')

def set_control_mode(mode):
    with open(yaml_path, 'r') as f:
        content = f.read()
    
    # Regex to find control_mode and replace its value
    content = re.sub(r'control_mode:\s*\d+', f'control_mode: {mode}', content)
    
    with open(yaml_path, 'w') as f:
        f.write(content)
    print(f"Set control_mode to {mode}")

def cleanup_dangling():
    # Force kill any lingering processes to avoid conflicts in the next run
    subprocess.run(["pkill", "-f", "gz"], check=False)
    subprocess.run(["pkill", "-f", "ros2"], check=False)
    subprocess.run(["pkill", "-f", "go2_refs"], check=False)
    subprocess.run(["pkill", "-f", "ruby"], check=False) # sometimes used by gz
    time.sleep(2)

def main():
    print("Starting automated experiments...")
    cleanup_dangling()

    run_sourced("colcon build")

    for mode in control_modes:
        print(f"\n{'='*50}\nStarting Experiment for Control Mode {mode}\n{'='*50}")
        
        set_control_mode(mode)
        
        print("Rebuilding go2_description...")
        run_sourced("colcon build --packages-select go2_description")
        
        print("Launching Gazebo simulation...")
        sim_proc = run_sourced("ros2 launch go2_gzsim go2.launch.py", bg=True)
        
        # Wait for gazebo and controllers to load properly
        print("Waiting 15 seconds for Gazebo to load...")
        time.sleep(15)
        
        bag_dir = f"{workspace_dir}/Data/Sim/Control_mode_{mode}"
        os.makedirs(bag_dir, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        bag_name = f"{bag_dir}/rosbag2_{timestamp}"
        
        print("Starting go2_refs (Reference Generator)...")
        refs_proc = run_sourced("ros2 run go2_refs go2_refs", bg=True)
        
        # Wait a moment for refs to start publishing /lowcmd so /lowstates creates its publisher
        time.sleep(2)
        
        print(f"Starting rosbag record at: {bag_name}")
        bag_proc = run_sourced(f"ros2 bag record -a -o {bag_name}", bg=True)
        
        # The trajectory takes: 5s + 15s + 15s + 1.5s + 1.5s = 38 seconds. 
        # Waiting 60 seconds to guarantee it records the full movement and steady state.
        print("Waiting 60 seconds for the trajectory to complete...")
        time.sleep(60)
        
        print("Stopping recording, refs and simulation gracefully...")
        # Send SIGINT to the process groups to kill cleanly (allows rosbag to save metadata)
        try:
            os.killpg(os.getpgid(bag_proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        time.sleep(2)
        
        try:
            os.killpg(os.getpgid(refs_proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
            
        try:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        
        print("Waiting 5 seconds for cleanup...")
        time.sleep(5)
        
        cleanup_dangling()
        
        print(f"Experiment for Control Mode {mode} finished successfully.")

    print("\nAll experiments finished! You can now run the extraction script.")

if __name__ == "__main__":
    main()
