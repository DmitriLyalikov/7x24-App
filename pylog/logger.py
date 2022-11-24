'''
Author: Dmitri Lyalikov
This script will gather ESP32 logging data from standard output at runtime over
the idf.py monitor utility and output this data and some basic statistics to a .csv file
'''

import sys
import re
import shutil
import subprocess
import time
import pandas as pd
from pathlib import path 
import os

TEMP_TAG = "Temperature {LM35}"
FLOW_TAG = "Flow Sensor {GR-2048}"
class logger:
    def __init__(self, test_duration: int, output_file: str, comport: str):
        self.test_invoke_cmd = f"idf.py monitor {comport}"
        self.test_exit_cmd = "^]"
        self.output_file = output_file
        self.path = os.getcwd()
        self.log_file = "monitor_log.txt"

        # Convert seconds to milliseconds
        self.duration = test_duration * 60 
        self.test_data = {'time': [], 'temp': [], 'flow': []}
        self.start_test()

    # Start monitoring 
    def start_monitor(self):
        os.chdir("..")
        # Start monitoring ESP-device and save output to temp file
        monitor = subprocess.Popen(['python3', self.test_invoke_cmd], stdout=self.log_file, shell=True)
        monitor.communicate()
        time.sleep(self.duration)
        monitor.terminate()
        # move everything back into working directory
        shutil.move(os.path.join(os.getcwd(), self.log_file), self.path)
        os.chdir(self.path)

    def parse_monitor(self):
        time_eq = 0
        log_file = open(self.log_file, "r")
        for line in log_file:
            if not self.test_data["time"]:
                # parse first time value and set to time_eq self.test_data[time][0]

            if TEMP_TAG in line:
                # subtract time_eq from time and append to self.test_data
                self.test_data 
        
            elif FLOW_TAG in line:
                # subtract time_e  from time and append to self.test_data



    def build_output(self):
        # add title, info, duration
        # construct from dict to CSV

        # build plots 



if __name__ == "__main__":
    print("************* 7x24 ESP32 Test Logging Automation ************* \n")
    print(f"Got Device COM port: {sys.argv[1]} \nGot File Name: {sys.argv[2]} \nGot Test Duration (Minutes): {sys.argv[3]}")
    test = logger(test_duration=int(sys.argv[3]), output_file=sys.argv[2], comport=sys.argv[1])