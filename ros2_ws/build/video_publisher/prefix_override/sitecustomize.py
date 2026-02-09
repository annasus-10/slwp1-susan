import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/slwp1/work/testing123/susan/ros2_ws/install/video_publisher'
