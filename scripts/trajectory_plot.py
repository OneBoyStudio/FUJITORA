import matplotlib.pyplot as plt
import pandas as pd
import os

current_dir = os.path.dirname(__file__)
log_path = os.path.abspath(os.path.join(current_dir, "..", "logs", "trajectory.csv"))

trajectory_logs = pd.read_csv(log_path)
plt.plot(trajectory_logs['time'], trajectory_logs['theta'])
plt.show()