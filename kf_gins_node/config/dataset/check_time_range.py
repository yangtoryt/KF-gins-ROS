import numpy as np

def read_time_range(filepath, col=0):
    times = []
    with open(filepath, 'r') as f:
        for line in f:
            if line.strip() == "":
                continue
            try:
                parts = line.strip().split()
                t = float(parts[col])
                times.append(t)
            except:
                continue
    if not times:
        return None, None
    return min(times), max(times)

if __name__ == "__main__":
    imu_file = "/home/yang/KF-ROS/ros_ws/src/kf_gins_node/config/dataset/Leador-A15.txt"
    gnss_file = "/home/yang/KF-ROS/ros_ws/src/kf_gins_node/config/dataset/GNSS-RTK.txt"

    imu_min, imu_max = read_time_range(imu_file, col=0)
    gnss_min, gnss_max = read_time_range(gnss_file, col=0)

    print("IMU time range:", imu_min, "→", imu_max)
    print("GNSS time range:", gnss_min, "→", gnss_max)

    # 建议 starttime
    starttime = max(imu_min, gnss_min)
    print("Recommended starttime:", starttime)
