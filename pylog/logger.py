'''
Author: Dmitri Lyalikov
This script will gather ESP32 logging data from standard output at runtime over
the idf.py monitor utility and output this data and some basic statistics to a .csv file
'''

import sys
import subprocess
import time
import pandas as pd
from pathlib import path 
import os

class logger:
    def __init__(self, test_duration: int, output_file: str, comport: str):
        self.test_invoke_cmd = f"idf.py monitor {comport}"
        self.test_exit_cmd = "^]"
        self.output_file = output_file
        self.path = os.getcwd()

        # Convert seconds to milliseconds
        self.duration = test_duration * 60 
        self.test_data = {'time': [], 'temp': [], 'flow': []}
        self.start()

    def start_monitor(self):
        os.chdir("..")
        # Sta
        subprocess.check_output(['python3', self.test_invoke_cmd])
        time.sleep()

    def run_test(self):

    def parse_monitor(self):

    def build_output(self):



if __name__ == "__main__":
    print("************* 7x24 ESP32 Test Logging Automation ************* \n")
    print(f"Got Device COM port: {sys.argv[1]} \nGot File Name: {sys.argv[2]} \nGot Test Duration (Minutes): {sys.argv[3]}")
    test = logger(test_duration=int(sys.argv[3]), output_file=sys.argv[2], comport=sys.argv[1])