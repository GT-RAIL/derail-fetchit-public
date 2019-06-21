"""Set up paths for affordance_net library"""

import os.path as osp
import sys


def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)


# Add lib to PYTHONPATH
this_dir = osp.dirname(__file__)
lib_path = osp.join(this_dir, '../../lib/affordance_net/lib')
add_path(lib_path)
