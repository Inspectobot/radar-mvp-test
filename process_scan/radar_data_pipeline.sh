#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR

echo -n "Starting radar data pipeline process..."

python3 $SCRIPT_DIR/aio_simple_sfcw_radar_scan.py
