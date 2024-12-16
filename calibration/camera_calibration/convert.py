import numpy as np

# Load the saved file using np.load
data = np.load('/home/lucvt001/camera_calibration/calibration_data.npz')

# Extract the arrays from the loaded file
array1 = data['mtx']
array2 = data['dist']

# Convert the arrays to text format using np.savetxt
np.savetxt('/home/lucvt001/camera_calibration/calibration_data.txt', np.column_stack((array1, array2)), delimiter=' ')