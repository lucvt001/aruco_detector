import numpy as np

# Path to the .npz file
file_path = '/home/lucvt001/camera_calibration/calibration_data.npz'

# Load the .npz file
data = np.load(file_path)

# Print out all the arrays inside the .npz file
for array_name in data.files:
    print(f"Array name: {array_name}")
    print(f"Array data:\n{data[array_name]}")
    print()