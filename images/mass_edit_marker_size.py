import yaml

def update_yaml_values(input_file, output_file):
    # Load the YAML file
    with open(input_file, 'r') as file:
        data = yaml.safe_load(file)

    # Update all parameter values to 2.0
    for key in data.keys():
        data[key] = 2.0

    # Save the updated YAML file
    with open(output_file, 'w') as file:
        yaml.dump(data, file)

if __name__ == "__main__":
    input_file = "/home/lucvt001/drone_ws/src/fiducial_detector/images/marker_size_draft.yaml"  # Replace with your input YAML file path
    output_file = "/home/lucvt001/drone_ws/src/fiducial_detector/images/marker_size_draft_updated.yaml"  # Replace with your desired output YAML file path

    update_yaml_values(input_file, output_file)
    print(f"Updated YAML file saved to {output_file}")