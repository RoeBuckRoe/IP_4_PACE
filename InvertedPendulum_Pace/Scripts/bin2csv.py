#!/usr/bin/env python
# Script for converting MARTe2 binary log files into CSV
#
# Usage: bin2csv.py <infile> [-o outfile]

import argparse
import os.path
import struct
from typing import Type
from dataclasses import dataclass


class Bin2CSVError(Exception):
    """Generic exception for all errors"""
    pass


@dataclass
class TypeDescriptor:
    name: str
    struct_format: str
    size: int


@dataclass
class SignalDescriptor:
    name: str
    type_descr: TypeDescriptor
    elements: int


standard_type_descriptors = {
    2048: TypeDescriptor('int32', 'i', 4),
    2052: TypeDescriptor('uint32', 'I', 4),
    4100: TypeDescriptor('uint64', 'Q', 8),
    1028: TypeDescriptor('uint16', 'H', 2),
    516: TypeDescriptor('uint8', 'B', 1)
}


def main(infile, outfile):
    """Convert the binary file into CSV"""
    with open(infile, 'rb') as fin:
        nsignals, = struct.unpack('<I', fin.read(4))
        print(f'Number of signals: {nsignals}')

        signals = []
        for i in range(nsignals):
            signal_type, signal_name, signal_elements = struct.unpack('<H32sI', fin.read(38))
            if signal_elements != 1:
                raise Bin2CSVError(f'Signal element count > 1 for signal {signal_name} not supported')
            
            signal_name = signal_name.rstrip(b'\x00').decode('utf-8')
            signals.append(SignalDescriptor(signal_name,
                                            standard_type_descriptors[signal_type], signal_elements))

        with open(outfile, 'w') as fout:
            header_row = '#' + \
                         ','.join(f'{signal.name} ({signal.type_descr.name})[{signal.elements}]' for signal in signals) + \
                         '\n'
            fout.write(header_row)

            row_size = sum(signal.type_descr.size for signal in signals)
            row_format = '<' + ''.join(signal.type_descr.struct_format for signal in signals)

            while True:
                raw_row = fin.read(row_size)
                if len(raw_row) < row_size:
                    break
                
                row_values = struct.unpack(row_format, raw_row)

                row = ','.join(str(value) for value in row_values) + '\n'
                fout.write(row)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('infile', type=str, help='Path to the input binary file')
    parser.add_argument('-o', '--outfile', help='Path to the output file')

    args = parser.parse_args()

    if args.outfile:
        outfile = args.outfile
    else:
        root, ext = os.path.splitext(args.infile)
        outfile = root + '.csv'
    
    main(args.infile, outfile)
    


