import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/only/workspace/wang/smartcar/10_axis_imu/imu_ws/install/wit_ros2_imu'
