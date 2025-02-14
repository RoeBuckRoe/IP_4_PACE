#!/usr/bin/env python
# Script for converting MARTe2 binary log files into CSV
#
# Usage: plot_csv.py <infile> [-i initial] [-f final]
# 
import argparse
import os.path
import sys

import matplotlib.pyplot as plt
import pandas
import numpy as np

def plot_raw_data(data_frame: pandas.DataFrame, intial: int, final: int, adc:int = 1, cleanup:bool = False):
    """Plot the raw ADC data between two points"""
    if adc == 1:
        raw_adc_values = data_frame['ADC1Data (uint16)[1]'].to_numpy()
    else:
        raw_adc_values = data_frame['ADC2Data (uint16)[1]'].to_numpy()

    scaled_raw_values = (raw_adc_values / 4095) * 3.3

    seconds = data_frame['ADCUTCUnixSeconds (uint32)[1]'].to_numpy()
    microseconds = data_frame['ADCUTCMicroseconds (uint32)[1]'].to_numpy()
    milliseconds = (seconds * 1_000_000 + microseconds) / 1_000
    validity = data_frame['TimestampedValidity (uint8)[1]'].to_numpy()
    
    utc_validity = microseconds < 1_000_000
    #utc_validity = milliseconds < (1.64925e12 + 7_000_000)
    adc_validity = scaled_raw_values < 3.3
    mask = np.logical_and(validity == 1, utc_validity)
    mask = np.logical_and(mask, adc_validity)

    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.set_ylabel('Voltage [V]')
    ax1.set_xlabel('Time [ms]')
    ax1.set_title('50 Hz mains voltage')

    if cleanup:
        milliseconds = milliseconds[mask]
        scaled_raw_values = scaled_raw_values[mask]

    ax1.plot(milliseconds, scaled_raw_values, 'b-', label='Raw ADC data (continuous)')
    ax1.plot(milliseconds, scaled_raw_values, 'r*', label='Raw ADC data')
    
    ax1.legend()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot data from a MARTe For Industry CSV log file')
    parser.add_argument('file', type=str, help='Path to the log file')
    parser.add_argument('--initial', type=int, help='Index of the first point to plot')
    parser.add_argument('--final', type=int, help='Index of the last point to plot')
    parser.add_argument('-c', '--cleanup', action='store_true', help='Clean up the plot by removing invalid data')

    args = parser.parse_args()

    if not os.path.isfile(args.file):
        print(f'Unable to find file {args.file}')
        sys.exit(1)

    data_frame = pandas.read_csv(args.file)

    if not args.initial:
        initial = 0
    else:
        initial = args.initial
    
    if not args.final:
        final = data_frame.shape[0]
    else:
        final = args.final
        
    plot_raw_data(data_frame, initial, final, cleanup=args.cleanup)
