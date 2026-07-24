import matplotlib.pyplot as plt
import pandas as pd
import os

from scipy.spatial.transform import Rotation as r

current_dir = os.path.dirname(__file__)
log_path = os.path.abspath(os.path.join(current_dir, "..", "logs", "trajectory.csv"))

trajectory_logs = pd.read_csv(log_path)

quaternion_data = trajectory_logs[['q2', 'q3', 'q4', 'q1']].to_numpy()
angles = r.from_quat(quaternion_data).as_euler('xyz', degrees=True)

fig, axes = plt.subplots(nrows=3, ncols=1)

axes[0].plot(trajectory_logs['time'], angles[:, 0])
axes[0].set_title('X')

axes[1].plot(trajectory_logs['time'], angles[:, 1])
axes[1].set_title('y')

axes[2].plot(trajectory_logs['time'], angles[:, 2])
axes[2].set_title('Z')

#plt.plot(trajectory_logs['time'], trajectory_logs['z'])
plt.show()