#!/usr/bin/env python3

# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import sys, os, webbrowser
# set python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../src/'))

from uploader.run import OpenstackObjectUploader
from uploader.model import CONFIG

# ---------------------------------------------------------
# The main program for running the application
if __name__ == '__main__':
    NODE_NAME = 'openstack_object_uploader'
    the_agent = OpenstackObjectUploader()
    DASH_HOST = CONFIG.get('uploader.web.host')
    DASH_PORT = CONFIG.get('uploader.web.host')
    if CONFIG.get('uploader.web.launch_browser', False):
        URL = f'http://{DASH_HOST}:{DASH_PORT}'
        webbrowser.open(URL)