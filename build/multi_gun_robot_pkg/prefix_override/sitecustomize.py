import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rokey/b4_multi_robot_ws/install/multi_gun_robot_pkg'
