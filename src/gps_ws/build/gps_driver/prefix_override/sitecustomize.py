import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/only/workspace/wang/smartcar/gps_module/gps_ws/install/gps_driver'
