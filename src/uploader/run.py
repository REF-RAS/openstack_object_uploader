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

# import libraries
import sys, os, signal, time, threading, webbrowser, subprocess, traceback
from datetime import datetime
# ros modules
import rospy, rosnode
# project modules
from tools.yaml_tools import YamlConfig
from tools.logging_tools import logger
import tools.file_tools as file_tools
import uploader.model as model
from uploader.model import DAO, CONFIG, STATE
from uploader.web.dashapp_top import DashAppTop
from watchdog.observers import Observer
from watchdog.events import LoggingEventHandler, FileSystemEventHandler, FileSystemEvent
import openstack, openstack.exceptions, keystoneauth1, keystoneauth1.exceptions
from openstack.config import loader

# The custom class for handling events of the watchdog modeul
class CustomFileSystemEventHandler(FileSystemEventHandler):
    def __init__(self, filestore_path):
        super(CustomFileSystemEventHandler, self,).__init__()
        self.filestore_path = filestore_path
        self.uploader_delay = CONFIG.get('uploader.delay', 30)
        # setup ignore list
        self.ignore_suffix = CONFIG.get('uploader.ignore.suffix', None)
        if self.ignore_suffix is not None and type(self.ignore_suffix) == str:
            self.ignore_suffix = [self.ignore_suffix]
        self.ignore_prefix = CONFIG.get('uploader.ignore.prefix', None)
        if self.ignore_prefix is not None and type(self.ignore_prefix) == str:
            self.ignore_prefix = [self.ignore_prefix]  

    def is_in_ignore_lists(self, filename:str) -> bool:
        if self.ignore_suffix is not None and type(self.ignore_suffix) in (tuple, list):
            for prefix in self.ignore_suffix:
                if filename.endswith(prefix):
                    return True
        if self.ignore_prefix is not None and type(self.ignore_prefix) in (tuple, list):
            for prefix in self.ignore_prefix:
                if filename.startswith(prefix):
                    return True        
        return False

    # The callback function when a create event occurs
    def on_created(self, event:FileSystemEvent):
        pass
    # the callback function when a file is modified
    def on_modified(self, event:FileSystemEvent):
        if not event.is_directory:
            # if the event is from a modified file
            local_path = event.src_path
            filename = file_tools.get_filename(local_path)
            if self.is_in_ignore_lists(filename):
                return
            parent_path = file_tools.get_parent(local_path)
            sub_path = parent_path[len(self.filestore_path) + 1:]  # the sub_path cannot start with '/' for os.path.join to work
            remote_filename = os.path.join(sub_path, filename)
            DAO.add_upload_file(local_path, filename, remote_filename, int(time.time()) + CONFIG.get('uploader.delay', self.uploader_delay))
    # the callback function when a file is deleted
    def on_deleted(self, event:FileSystemEvent):
        if not event.is_directory:
            # if the event is from a file
            local_path = event.src_path
            DAO.remove_upload_file(local_path)

class OpenstackObjectUploader(object):
    def __init__(self):
        # the configuration and other system parameters
        self.application_config:YamlConfig = CONFIG

        # the parameters concerning the uploading operation
        self.uploader_delay = CONFIG.get('uploader.delay', 30)
        self.uploader_max_error_count = CONFIG.get('uploader.error_count.max', 5)
        # setup connection to openstack cloud
        # reference: https://docs.openstack.org/openstacksdk/latest/user/guides/connect_from_config.html
        self.cloud_name = CONFIG.get('uploader.filestore.cloud', 'openstack')
        config = loader.OpenStackConfig()
        try:
            self.cloud = openstack.connect(cloud=self.cloud_name)
        except openstack.exceptions.ConfigException as e:
            self.cloud = None
            logger.error(f'{type(self).__name__}: The application is unable to locate the application credential file "clouds.yaml".')
            logger.warning(f'{type(self).__name__}: Ensure that a clouds.yaml file is obtained from the openstack cloud provider.')
            logger.warning(f'{type(self).__name__}: If you have a valid clouds.yaml file, make sure it is in the current directory or ~/.config/openstack and only one cloud.yaml file.')
            logger.warning(f'{type(self).__name__}: Terminate the application and fix the problem according to the README.md file.')
            sys.exit(1)

        # prepare operation mode
        self.operation_mode = rospy.get_param('mode', None)
        if self.operation_mode is None or self.operation_mode == "":
            self.operation_mode = CONFIG.get('uploader.mode', 'web')
        # create lock for synchronization
        self.state_lock = threading.Lock()
        self.log_lock = threading.Lock()
        # create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)
        rospy.on_shutdown(self.cb_shutdown)
        # setup watchdog
        self.filestore_local = CONFIG.get('uploader.filestore.local', None)
        self.filestore_container = CONFIG.get('uploader.filestore.cloud.container', None)
        if self.filestore_local is None:
            logger.warning(f'{type(self).__name__}: The local filestore location is not defined in the config file')
            sys.exit(1)
        if not os.path.isdir(self.filestore_local):
            logger.info(f'{type(self).__name__}: The local filestore location {self.filestore_local} does not exist, create the directory')
            os.makedirs(self.filestore_local, exist_ok=True)
        if self.filestore_container is None:
            logger.warning(f'{type(self).__name__}: The cloud filestore container is not defined in the config file')
            sys.exit(1)   
        # attempt to make a connection to the container
        try:
            if self.cloud is not None:
                self.container = self.cloud.object_store.create_container(self.filestore_container, is_content_type_detected=True)
                logger.info(f'{type(self).__name__}: Connected to (and if needed created) the container "{self.filestore_container}"')
        except keystoneauth1.exceptions.http.Unauthorized as e:
            logger.error(f'{type(self).__name__}: The application is unable to authenticate using the credentials in "clouds.yaml".')
            logger.warning(f'{type(self).__name__}: Ensure that the file is current and valid to the cloud server.')
            logger.warning(f'{type(self).__name__}: The access right should include granting access to an object store.')
            logger.warning(f'{type(self).__name__}: Terminate the application and fix the problem according to the README.md file.')
            sys.exit(1)          

        # start the watchdog 
        self.watchdog_thread = self.run_watchdog(self.filestore_local)
        # start the uploader thread
        self._to_stop_uploader = False
        self.uploader_thread = threading.Thread(target=self.run_uploader)
        self.uploader_thread.start()
        logger.info(f'{type(self).__name__}: Started one-way sync from the local "{self.filestore_local}" to the container "{self.filestore_container}".')

        # starts the web server    
        if self.operation_mode == 'web':
            # create the dash application and start it and block the thread
            self.dash_app_operator = DashAppTop()
            self.dash_app_operator.start()
        elif self.operation_mode == 'headless':
            pass
        else:
            logger.warning(f'{type(self).__name__} (__init__): invalid uploader.mode ("{self.operation_mode}") in config')
            sys.exit(0)

        # spin the application if the mode is headless
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.watchdog_thread.stop()
            self.uploader_thread_stop()
        self.watchdog_thread.join()
        self.uploader_thread.join()

    # callback when interrupt signal is received
    def stop(self, *args, **kwargs):
        logger.info(f'{type(self).__name__}: The ros node is being stopped')
        self.watchdog_thread.stop()
        self.uploader_thread_stop()
        sys.exit(0)

    # callback when the ros receives a shutdown signal
    def cb_shutdown(self):
        logger.info(f'{type(self).__name__}: The ros node is being shutdown')

    # callback from the GUI console
    def _console_callback(self, event, *args):
        with self.state_lock:
            pass
        
    # configure and execute the watchdog to monitor the filestore
    def run_watchdog(self, filestore_path):
        event_handler = CustomFileSystemEventHandler(filestore_path)
        observer = Observer()
        observer.schedule(event_handler, filestore_path, recursive=True)
        observer.start()
        return observer

    # an internal function for stopping the uploader thread
    def uploader_thread_stop(self):
        self._to_stop_uploader = True
    
    # executes the uploader thread
    def run_uploader(self):
        # clear the upload queue
        DAO.clear_upload_queue()
        while not self._to_stop_uploader:
            time.sleep(1.0)
            # if the google drive connection is not ready, change the state to ERROR
            if self.cloud is None:
                model.STATE.update(model.SystemStates.ERROR)
                continue
            # check if there is a file to be uploaded
            next_to_upload = DAO.query_next_upload_within_error(limit=1, max_error_count=self.uploader_max_error_count)
            if len(next_to_upload) == 0:
                continue
            # starts the uploading procedure
            # compute the parameters for uploading including the local and remote path
            model.STATE.set_var('upload', next_to_upload[0])
            local_path = next_to_upload[0]['local_path']
            remote_path = next_to_upload[0]['remote_path']
            file_size = (os.stat(local_path).st_size)
            start_time = time.time()
            logger.info(f'{type(self).__name__} (run_uploader): Uploading file {local_path} to {remote_path}')
            model.STATE.update(model.SystemStates.UPLOADING)
            try: 
                self.cloud.object_store.upload_object(container=self.container, name=remote_path, filename=local_path)
                # compute the upload time
                upload_duration = int((time.time() - start_time) * 1000)
                # update the database
                DAO.remove_upload_file(local_path)
                DAO.add_upload_record(local_path, file_size, upload_duration)
                logger.info(f'{type(self).__name__} (run_uploader): Uploading file SUCCESSFUL ({file_size / 1000} KB in {upload_duration / 1000:.2f} s)')
                model.STATE.update(model.SystemStates.READY)
                continue                
            except IOError as e:
                logger.error(f'{type(self).__name__} (run_uploader): Failed to read file from {local_path}')
            except Exception as e:
                logger.warning(f'{type(self).__name__} (run_uploader): Error: {traceback.format_exc()}')
            # handle upload errors
            DAO.error_delay_upload_timestamp(local_path, int(time.time()) + (self.uploader_delay * (next_to_upload[0]['error_count'] + 2)))
            logger.info(f'{type(self).__name__} (run_uploader): Uploading file FAILED')
            if next_to_upload[0]['error_count'] >= self.uploader_max_error_count:
                logger.info(f'{type(self).__name__} (run_uploader): Maximum error reached for uploading {local_path}')
            model.STATE.update(model.SystemStates.READY)

# ---------------------------------------------------------
# The main program for running the application
if __name__ == '__main__':
    NODE_NAME = 'openstack_object_uploader'
    rospy.init_node(NODE_NAME)
    the_agent = OpenstackObjectUploader()
    DASH_HOST = CONFIG.get('uploader.web.host')
    DASH_PORT = CONFIG.get('uploader.web.host')
    if CONFIG.get('uploader.web.launch_browser', False):
        URL = f'http://{DASH_HOST}:{DASH_PORT}'
        webbrowser.open(URL)

