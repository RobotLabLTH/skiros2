#!/usr/bin/env python
import os
import sys
import rospy
import pytest

def get_output_file():
    for arg in sys.argv:
        if arg.startswith('--gtest_output'):
            return arg.split('=xml:')[1]

    raise RuntimeError('No output file has been passed')

if __name__ == '__main__':
    output_file = get_output_file()
    test_module = rospy.get_param('test_module')
    runner_path = os.path.dirname(os.path.realpath(__file__))
    module_path = os.path.join(runner_path, test_module)
    args = [module_path,'--junitxml={}'.format(output_file)]
    if '--text' in sys.argv:
        args.append('-s')
    sys.exit(
        pytest.main(args)
        )
