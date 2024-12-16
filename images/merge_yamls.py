import yaml

def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def save_yaml(data, file_path):
    with open(file_path, 'w') as file:
        yaml.dump(data, file)

def merge_yaml_files(position_file, size_file, output_file):
    # Load the YAML files
    positions = load_yaml(position_file)
    sizes = load_yaml(size_file)

    # Merge the data
    merged_data = {}
    for marker_id, pos in positions.items():
        if marker_id in sizes:
            merged_data[marker_id] = {
                'x': pos[0],
                'y': pos[1],
                'size': sizes[marker_id]
            }

    # Save the merged data to a new YAML file
    save_yaml(merged_data, output_file)

if __name__ == "__main__":
    position_file = "/home/lucvt001/drone_ws/src/fiducial_detector/images/landing_pad_scaled.yaml"  # Replace with your positions YAML file path
    size_file = "/home/lucvt001/drone_ws/src/fiducial_detector/images/marker_size.yaml"  # Replace with your sizes YAML file path
    output_file = "/home/lucvt001/drone_ws/src/fiducial_detector/images/landing_pad_final.yaml"  # Replace with your desired output YAML file path

    merge_yaml_files(position_file, size_file, output_file)
    print(f"Merged YAML file saved to {output_file}")