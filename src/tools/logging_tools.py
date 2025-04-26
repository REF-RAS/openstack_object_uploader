# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import logging, time

# -- The custom logger for the task trees package
class CustomFormatter(logging.Formatter):
    """ The custom logger class for the package
    :meta private:
    """
    grey = '\x1b[38;20m'
    cyan ='\x1b[36;20m'
    yellow = '\x1b[33;20m'
    red = '\x1b[31;20m'
    bold_red = '\x1b[31;1m'
    reset = '\x1b[0m'
    # format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)'
    format = '[%(levelname)s] [%(created)16f]: %(message)s'
    FORMATS = {
        logging.DEBUG: cyan + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }
    def format(self, record):
        time_format = "%H:%M:%S %f"
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt, datefmt=time_format)
        return formatter.format(record)
    def formatException(self, exc_info):
        result = super().formatException(exc_info)
        return repr(result)

# Internal function that intializes the logger
def get_logger(name='global', level=logging.INFO, silent:bool=False, logging_file=None, logging_file_level=None):
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = False
    if not logger.handlers:
        if not silent or logging_file is None:
            ch = logging.StreamHandler()
            ch.setFormatter(CustomFormatter())
            ch.setLevel(level)
            logger.addHandler(ch)
        if logging_file:
            logging_file_level = level if logging_file_level is None else logging_file_level
            fh = logging.FileHandler(logging_file)
            fh.setLevel(logging_file_level)
            logger.addHandler(fh)
    return logger

