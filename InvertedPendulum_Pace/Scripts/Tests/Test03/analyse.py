#!/usr/bin/env python

# Script to analyse the log file for Test 3 ("Resample and sync to GPS")

import argparse
import os.path
import sys

import pandas
import numpy as np

def message_count_check(data_frame: pandas.DataFrame):
    """Check how often messages are lost"""
    stm32_rx_message_count = data_frame['#STM32MessageCount (uint32)[1]'].to_numpy()
    delta_stm32_rx_message_count = np.diff(stm32_rx_message_count)
    message_count = stm32_rx_message_count[-1]

    print(f'Minimum change in rx message count {np.min(delta_stm32_rx_message_count)}')
    print(f'Maximum change in rx message count {np.max(delta_stm32_rx_message_count)}')

    messages_lost = np.sum(delta_stm32_rx_message_count - 1)
    print(f'Number of rx messages discarded by the application: {messages_lost:,} ({(messages_lost/message_count)*100:.2f}%)')
    print(f'Total number of messages received by the application: {message_count:,}')

def message_rx_time_variance(data_frame: pandas.DataFrame):
    """Check the variance in message rx time"""
    stm32_message_rx_time = data_frame['STM32MessageRxTime (uint64)[1]'].to_numpy()
    delta_rx_time = np.diff(stm32_message_rx_time) / 1_000

    print(f'Mean time between messages: {np.mean(delta_rx_time):,.3f} us')
    print(f'Std. deviation of time between messages: {np.std(delta_rx_time):,.3f} us')
    print(f'Minimum time between messages: {np.min(delta_rx_time):,.3f} us')
    print(f'Maximum time between messages: {np.max(delta_rx_time):,.3f} us (at index {np.argmax(delta_rx_time)})')

def dac_latency(data_frame: pandas.DataFrame):
    """Check the variation in latency through the application"""
    stm32_message_rx_time = data_frame['STM32MessageRxTime (uint64)[1]'].to_numpy()
    stm32_message_tx_time = data_frame['STM32MessageTxTime (uint64)[1]'].to_numpy()
    delta_time = (stm32_message_tx_time[1:] - stm32_message_rx_time[:-1]) / 1_000

    print(f'Mean DAC latency: {np.mean(delta_time):,.3f} us')
    print(f'Std. deviation of DAC latency: {np.std(delta_time):,.3f} us')
    print(f'Minimum time between Rx and Tx: {np.min(delta_time):,.3f} us')
    print(f'Maximum time between Rx and Tx: {np.max(delta_time):,.3f} us (at index {np.argmax(delta_time)})')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Analyse test 3 data')
    parser.add_argument('file', type=str, help='Path to the test 3 logfile')
    args = parser.parse_args()

    if not os.path.isfile(args.file):
        print(f'Unable to find file {args.file}')
        sys.exit(1)
    
    print(f'Analysing contents of file {args.file}')

    data_frame = pandas.read_csv(args.file)

    print('*********************************')
    message_count_check(data_frame)
    print('*********************************')
    message_rx_time_variance(data_frame)
    print('*********************************')
    dac_latency(data_frame)
    

