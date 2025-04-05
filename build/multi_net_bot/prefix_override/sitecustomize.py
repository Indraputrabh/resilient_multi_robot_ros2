import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/indra/RESILIENT_MULTI_ROBOT_ROS2/install/multi_net_bot'
