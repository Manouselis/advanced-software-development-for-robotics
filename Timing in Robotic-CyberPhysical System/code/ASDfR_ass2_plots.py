import numpy as np
import matplotlib.pyplot as plt

# Load data from file
# Tip: Change directory to the place of the dataframe or the code won't work
time_arr_ras_n = np.loadtxt('C:/Users/s3084493/OneDrive - University of Twente/MSc Robotics/Quarter 3/Advanced Software Development for Robotics/Assignments/Assignment 2/raspberry_1_time_arr.txt')
time_arr_ras_c4_m2 = np.loadtxt('C:/Users/s3084493/OneDrive - University of Twente/MSc Robotics/Quarter 3/Advanced Software Development for Robotics/Assignments/Assignment 2/raspberry_3_c_4_m_2.txt')
time_arr_ras_c4_m6 = np.loadtxt('C:/Users/s3084493/OneDrive - University of Twente/MSc Robotics/Quarter 3/Advanced Software Development for Robotics/Assignments/Assignment 2/raspberry_2_c_4_m_6.txt')

def plot_execution_time(time_arr, details):
    # compute the mean and standard deviation of the execution time
    mean_time = np.mean(time_arr)
    std_dev = np.std(time_arr)

    execution_time = time_arr

    # plot the 1ms line
    plt.axhline(y=1, color='k', linestyle='--', label='1 ms')

    # plot the mean as a horizontal line
    plt.axhline(y=mean_time, color='r', label='Mean')

    # plot the standard deviation as an error bar
    plt.errorbar(0, mean_time, yerr=std_dev, fmt='o', color='g', label='Standard Deviation')

    plt.plot(execution_time, color='b', label='Execution Time')

    # plot the jitter as a line plot
    plt.plot(execution_time - 1, color='y', label='Jitter')

    plt.ylim(0, 2.5)
    plt.xlabel("Iteration")
    plt.ylabel("Execution Time (ms)")
    plt.title("Execution Time of Loop Iterations " + details)
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncols=3)
    plt.savefig('execution_time_plot_' + details + '.png', dpi=300, bbox_inches='tight')
    plt.legend()
    plt.show()

plot_execution_time(time_arr_ras_n, details="without Stress")
plot_execution_time(time_arr_ras_c4_m2, details="with Stress c4 m2")
plot_execution_time(time_arr_ras_c4_m6, details="with Stress c4m6")


# Define function to plot execution time
def histogram(time_arr, details, remove_first_row):
    # remove the first row if needed
    if remove_first_row:
        time_arr = time_arr[1:]

    # Subtract 1 ms from each value to calculate jitter
    jitter = time_arr - 1

    # calculate and display the histogram of the jitter. Use bins of width 0.01 ms
    plt.hist(jitter, bins=np.arange(0, 1, 0.01))
    plt.xlabel("Jitter (ms)")
    plt.ylabel("Count")
    plt.title("Histogram of Jitter " + details)
    plt.savefig('histogram_jitter_' + details + '.png', dpi=300, bbox_inches='tight')
    plt.show()

histogram(time_arr_ras_n, details="without Stress", remove_first_row=0)
histogram(time_arr_ras_c4_m2, details="with Stress c4 m2", remove_first_row=0)
histogram(time_arr_ras_c4_m6, details="with Stress c4m6", remove_first_row=0)

