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

# 在 check_time_range.py 中添加（替换原 main 函数）
if __name__ == "__main__":
    imu_file = "/home/yang/KF-ROS/ros_ws/src/kf_gins_node/config/dataset/Leador-A15.txt"
    gnss_file = "/home/yang/KF-ROS/ros_ws/src/kf_gins_node/config/dataset/GNSS-RTK.txt"
    times = []
    with open(imu_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if not parts or parts[0].startswith('#'):
                continue
            try:
                t = float(parts[0])
                times.append(t)
            except:
                continue
    # 检查相邻时间戳差值
    for i in range(1, len(times)):
        dt = times[i] - times[i-1]
        if dt <= 0:
            print(f"异常时间戳：第{i}行，dt={dt}（重复或倒序）")
    print("时间戳检查完成")
    # 在check_time_range.py的main函数末尾添加
    imu_min, imu_max = read_time_range(imu_file)
    gnss_min, gnss_max = read_time_range(gnss_file)
    print(f"IMU时间范围: {imu_min:.3f} ~ {imu_max:.3f}")
    print(f"GNSS时间范围: {gnss_min:.3f} ~ {gnss_max:.3f}")
    overlap_start = max(imu_min, gnss_min)
    overlap_end = min(imu_max, gnss_max)
    print(f"重叠时间范围: {overlap_start:.3f} ~ {overlap_end:.3f}")
    if overlap_end - overlap_start < 10:  # 至少10秒重叠
        print("警告：IMU与GNSS数据重叠时间过短！")
