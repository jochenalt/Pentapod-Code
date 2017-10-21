#!/bin/bash
cpufreq-set -r -g ondemand
/bin/stty -F /dev/ttyS1 raw 230400 cs8 clocal -cstopb
