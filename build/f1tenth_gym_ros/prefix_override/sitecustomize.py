import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tripan/f1tenthsim_ws/f1tenth/install/f1tenth_gym_ros'
