import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cewan/Automated-Tour-Car/ws_lidar/install/gps_path_tracking'
