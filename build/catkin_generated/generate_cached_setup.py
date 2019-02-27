# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/kinetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/kinetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
<<<<<<< HEAD
    for workspace in "/home/dawsonc/e190_ws/devel;/opt/ros/kinetic".split(';'):
=======
    for workspace in "/home/peter/190_ws/devel;/home/peter/catkin_ws/devel;/opt/ros/kinetic".split(';'):
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

<<<<<<< HEAD
code = generate_environment_script('/home/dawsonc/e190_ws/devel/env.sh')

output_filename = '/home/dawsonc/e190_ws/build/catkin_generated/setup_cached.sh'
=======
code = generate_environment_script('/home/peter/190_ws/devel/env.sh')

output_filename = '/home/peter/190_ws/build/catkin_generated/setup_cached.sh'
>>>>>>> f71165a0ade14f78aa727a313e9ebe52fa57ba0b
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
