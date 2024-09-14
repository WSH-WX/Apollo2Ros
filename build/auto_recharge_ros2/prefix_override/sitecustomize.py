import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wsh/wheeltec/wheeltec_ros2/install/auto_recharge_ros2'
