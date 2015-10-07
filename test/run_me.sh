#!/bin/bash

gcc test.c -o test -lm

./test > outfile
./plotter.py
