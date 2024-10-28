'''
Copyright (C) 2020 Clement Chen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

import logging
import logging.handlers
import os
import inspect
import traceback

class LogHandler:
    '''
        Generate and handle log file and log message\n
        for initial logging_level, please input one of these ["debug", "info", "warring", "error", "critical", "none"]\n
        
        ######### Please Note #############\n
        if there is a child log and if you don't want it to have multi times log.
        please set need_propagate=False \n
        Child log example: log_name= "parent.child"\n
        With the dot after "parent", the logger "child" will be considered as the child log of logger "parent"\n\n
        For more information, please see https://docs.python.org/3/library/logging.html

    '''
    def __init__(self, log_name, log_file_name, log_level="info", file_log_level="debug", need_propagate=True):
        self.level_dict = {
            "debug":logging.DEBUG,
            "info":logging.INFO,
            "warning":logging.WARNING,
            "error":logging.ERROR,
            "critical":logging.CRITICAL,
            "none":logging.CRITICAL + 1
        }
        self.logger = logging.getLogger(log_name)
        if not need_propagate:
            self.logger.propagate = False
        self.stream_handler = logging.StreamHandler()
        try:
            self.file_handler = logging.handlers.RotatingFileHandler(
                filename="./LOG/" + log_file_name + ".txt",
                maxBytes=int(1024*1024),
                backupCount=10,
                encoding="utf-8",
            )
        except FileNotFoundError:
            os.mkdir("./LOG")
            self.file_handler = logging.handlers.RotatingFileHandler(
                filename="./LOG/" + log_file_name + ".txt",
                maxBytes=int(1024*1024),
                backupCount=10,
                encoding="utf-8",
            )

        self.stream_handler.setFormatter(logging.Formatter(
            "[{asctime}] [{name}] [{levelname}]: {message}",
            style="{"
        ))
        self.file_handler.setFormatter(logging.Formatter(
            "[{asctime}] [{name}] [{levelname}]: {message}",
            style="{"
        ))
        self.logger.addHandler(self.stream_handler)
        self.logger.addHandler(self.file_handler)
        self.logger.setLevel(level=self.level_dict["debug"])
        self.stream_handler.setLevel(level=self.level_dict[log_level])
        self.file_handler.setLevel(level=self.level_dict[file_log_level])
    
#=========== Overload log method from logging ============
    def debug(self, message="", function_or_class=None):
        try:
            if function_or_class is None:
                self.logger.debug(f"{self.getCallerName()} : " + message)
            else:
                self.logger.debug(f"{function_or_class.__name__} : " + message)
        except:
            self.logger.exception()

    def info(self, message="", function_or_class=None):
        try:
            if function_or_class is None:
                self.logger.debug(f"{self.getCallerName()} : " + message)
            else:
                self.logger.info(f"{function_or_class.__name__} : " + message)
        except:
            self.logger.exception()

    def warning(self, message="", function_or_class=None):
        try:
            if function_or_class is None:
                self.logger.warning(f"{self.getCallerName()} : " + message)
            else:
                self.logger.warning(f"{function_or_class.__name__} : " + message)
        except:
            self.logger.exception()

    def error(self, message="", function_or_class=None):
        try:
            if function_or_class is None:
                self.logger.error(f"{self.getCallerName()} : " + message)
            else:
                self.logger.error(f"{function_or_class.__name__} : " + message)
        except:
            self.logger.exception()

    def critical(self, message="", function_or_class=None):
        try:
            if function_or_class is None:
                self.logger.critical(f"{self.getCallerName()} : " + message)
            else:
                self.logger.critical(f"{function_or_class.__name__} : " + message)
        except:
            self.logger.exception()

    def exception(self, message="", function_or_class=None):
        try:
            if function_or_class is None:
                self.logger.exception(f"{self.getCallerName()} : " + message)
            else:
                self.logger.exception(f"{function_or_class.__name__} : " + message)
        except:
            self.logger.exception()


    def getCallerName(self) -> str:
        return inspect.getouterframes(inspect.currentframe(), 2)[2][3]

#=========== Overload General Method ===============

    def removeHandler(self):
        '''
            MAKE SURE TO CALL THIS if the object that use this method will be recreate.\n
            Or, the memory would accamulate and grow faster and faster
        '''
        self.logger.removeHandler(self.file_handler)
        self.logger.removeHandler(self.stream_handler)

    def setLogLevel(self, log_level=None, file_log_level=None):

        if log_level in self.level_dict:
            self.stream_handler.setLevel(self.level_dict[log_level])
        if file_log_level in self.level_dict:
            self.file_handler.setLevel(self.level_dict[file_log_level])

