#!/bin/sh
# Start the Growatt simulator. Arguments are passed on, see: ./run.sh --help
cd "$(dirname "$0")" && exec python3 -m growatt_sim "$@"
