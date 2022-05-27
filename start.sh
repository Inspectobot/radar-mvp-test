#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

LD_LIBRARY_PATH=/opt/redpitaya/lib ${SCRIPT_DIR}/bin/radar_redis_driver
