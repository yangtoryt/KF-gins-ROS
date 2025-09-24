# check_imu.py
import numpy as np
f = "/home/yang/KF-ROS/ros_ws/src/kf_gins_node/config/dataset/Leador-A15.txt"
times = []
data = []
with open(f) as fh:
    for i,line in enumerate(fh,1):
        line=line.strip()
        if not line: continue
        toks = line.split()
        if len(toks) < 7:
            print("Line", i, "bad len", len(toks)); continue
        try:
            row = list(map(float, toks[:7]))
        except:
            print("Line", i, "parse error"); continue
        times.append(row[0])
        data.append(row[1:])
times = np.array(times)
data = np.array(data)
if np.isnan(data).any() or np.isinf(data).any():
    print("Found NaN/Inf in IMU data")
dts = np.diff(times)
print("min dt", dts.min(), "max dt", dts.max(), "median", np.median(dts))
# typical magnitude checks
angle_norm = np.linalg.norm(data[:,0:3], axis=1)
vel_norm   = np.linalg.norm(data[:,3:6], axis=1)
print("angle increment stats (rad): min, median, max:", angle_norm.min(), np.median(angle_norm), angle_norm.max())
print("delta-v stats (m/s): min, median, max:", vel_norm.min(), np.median(vel_norm), vel_norm.max())
