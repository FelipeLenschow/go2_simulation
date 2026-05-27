import os
import csv
import argparse
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def get_message_type(type_name):
    try:
        return get_message(type_name)
    except Exception as e:
        print(f"Error getting message type {type_name}: {e}")
        return None

def main():
    parser = argparse.ArgumentParser(description="Extract ROS 2 bags to CSV")
    parser.add_argument('--overwrite', '-o', action='store_true', help="Overwrite existing CSV files")
    args = parser.parse_args()

    base_dir = '/home/felipe/go2_simulation/Data'
    
    # Target motors (using indices 1 and 2 as requested, modify if you need 0-based index)
    motors = [1, 2] 
    
    target_topics = [
        '/go2_jointcontroller/JointControllerReferences',
        '/lowcmd',
        '/lowstate'
    ]

    # Find all rosbag directories
    bag_paths = []
    for root, dirs, files in os.walk(base_dir):
        if 'metadata.yaml' in files:
            bag_paths.append(root)

    if not bag_paths:
        print(f"No rosbag directories found in {base_dir}.")
        return

    # Prepare CSV columns
    fieldnames = ['timestamp']
    for m in motors:
        fieldnames.extend([
            f'ref_q_{m}',
            f'cmd_tau_{m}',
            f'state_q_{m}',
            f'state_tau_est_{m}'
        ])

    for bag_path in bag_paths:
        output_csv = os.path.join(os.path.dirname(bag_path), 'extracted_data.csv')
        if os.path.exists(output_csv) and not args.overwrite:
            print(f"Skipping {bag_path}, already extracted. Use --overwrite to regenerate.")
            continue
            
        print(f"Processing bag: {bag_path}")
        
        with open(output_csv, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            storage_id = 'sqlite3'
            meta_path = os.path.join(bag_path, 'metadata.yaml')
            if os.path.exists(meta_path):
                import yaml
                with open(meta_path, 'r') as f:
                    meta = yaml.safe_load(f)
                    try:
                        storage_id = meta['rosbag2_bagfile_information']['storage_identifier']
                    except KeyError:
                        pass
                        
            storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            
            reader = SequentialReader()
            try:
                reader.open(storage_options, converter_options)
            except Exception as e:
                print(f"Failed to open {bag_path}: {e}")
                continue

            topic_types = reader.get_all_topics_and_types()
            type_map = {t.name: t.type for t in topic_types}
            
            # Cache deserialization types
            msg_types = {}
            for t_name, t_type in type_map.items():
                if t_name in target_topics:
                    msg_type = get_message_type(t_type)
                    if msg_type:
                        msg_types[t_name] = msg_type
            
            # Maintain latest state (Zero-Order Hold)
            latest_state = {f: '' for f in fieldnames}
            first_t = None
            
            while reader.has_next():
                (topic, data, t) = reader.read_next()
                if topic not in target_topics:
                    continue
                
                if first_t is None:
                    first_t = t
                
                msg_type = msg_types.get(topic)
                if msg_type is None:
                    continue
                    
                msg = deserialize_message(data, msg_type)
                latest_state['timestamp'] = round((t - first_t) / 1e9, 4)
                updated = False
                
                try:
                    if topic == '/go2_jointcontroller/JointControllerReferences':
                        for m in motors:
                            if hasattr(msg, 'motor_cmd') and len(msg.motor_cmd) > m:
                                latest_state[f'ref_q_{m}'] = round(msg.motor_cmd[m].q, 4)
                                updated = True
                    elif topic == '/lowcmd':
                        for m in motors:
                            if hasattr(msg, 'motor_cmd') and len(msg.motor_cmd) > m:
                                latest_state[f'cmd_tau_{m}'] = round(msg.motor_cmd[m].tau, 4)
                                updated = True
                    elif topic == '/lowstate':
                        for m in motors:
                            if hasattr(msg, 'motor_state') and len(msg.motor_state) > m:
                                latest_state[f'state_q_{m}'] = round(msg.motor_state[m].q, 4)
                                latest_state[f'state_tau_est_{m}'] = round(msg.motor_state[m].tau_est, 4)
                                updated = True
                except AttributeError as e:
                    print(f"Attribute error on topic {topic}: {e}")
                    continue
                
                # Write row whenever we get a new message for these topics
                if updated:
                    writer.writerow(latest_state)

        print(f"Extraction complete for {bag_path}! Saved to {output_csv}")

    print("All extractions complete!")

if __name__ == '__main__':
    main()
