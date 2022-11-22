'''
Author: Dmitri Lyalikov
This script will gather ESP32 logging data from standard output at runtime over
the idf.py monitor utility and output this data and some basic statistics to a .csv file
'''

import pandas as pd
import os

class logger:
    def __init__(self, test_duration: int, output_file = "7x24_Sensor_Data"):
        self.output_file = output_file
        self.duration = test_duration
        self.start()

    def start(self):
        
    def run_test(self):

    def parse_monitor(self):

    def build_output(self):



if __name__ == "__main__":
    test = logger()