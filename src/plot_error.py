"""
We evaluate the accuracy of /rvc_odom by comparing its pose discrepancy with
 aother odometry topic. For this, we use /odom as ground truth and assess /rvc_odom's errors. 
 Then we use the first error as reference, use the first error as a reference and calculate 
 the relative error for the subsequent points, plot the data, and compute the necessary statistics.  

"""


import pandas as pd
import matplotlib.pyplot as plt

# Assuming 'data' is your DataFrame and it has been read from your csv file

# fileLocation = '/home/zuyuan/ros2_ws/mapping/foxy_with-scan'
# dataName = 'foxy-with-scan'
# plotTitle = 'Cartographer: Discrepancy between /rvc_odom and /odom with Continuous Scan'

fileLocation = '/home/zuyuan/ros2_ws/mapping/foxy_stopScan_Vel_Acc'
dataName = 'poseError'
plotTitle = 'Cartographer: Discrepancy between /rvc_odom and /odom with Scan Stopped'



fileName = fileLocation + '/' + dataName + '.csv'
data = pd.read_csv(fileName, names=['timestamp', 'error'])

# Convert timestamp to datetime and set it as index
data['timestamp'] = pd.to_datetime(data['timestamp'])
data.set_index('timestamp', inplace=True)

# Calculate the relative error
data['relative_error'] = data['error'] - data['error'].iloc[0]

# Plotting
plt.figure(figsize=(10,6))
plt.plot(data['relative_error'])
plt.title(plotTitle)
plt.xlabel('Timestamp')
plt.ylabel('Relative Error')
plt.grid(True)


saveFileName = fileLocation + '/' + dataName + '_error.png'
plt.savefig(saveFileName)
plt.show()


# Calculate statistics
statistics = data['relative_error'].describe()

# Add descriptive labels
statistics_labels = {
    'Count': 'Number of samples',
    'Mean': 'Average relative error',
    'Std': 'Standard deviation',
    'Min': 'Minimum relative error',
    '25%': '25th percentile',
    '50%': 'Median (50th percentile)',
    '75%': '75th percentile',
    'Max': 'Maximum relative error'
}

# Rename the index (statistical measure names)
statistics.rename(index=statistics_labels, inplace=True)

statisticsResults = fileLocation + '/' + dataName + '_error_statistics.csv'
# Save the results to a file (e.g., CSV or text file)
statistics.to_csv(statisticsResults, index=True)

print("Statistics saved successfully!")

print(statistics)
