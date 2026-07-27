#!/bin/bash
cd ~/src/sdr-workspace/test_runs || exit 1

trap 'kill $GNSS_PID $RECORDER_PID 2>/dev/null' EXIT

~/src/sdr-workspace/build/gnss_to_vrt --progress &
GNSS_PID=$!
sleep 2

~/src/sdr-workspace/build/iq_recorder --file "$1" --gnss --progress &
RECORDER_PID=$!
sleep 1

~/src/sdr-workspace/build/usrp_to_vrt --freq 98e6 --rate 2e6 --gain 40 \
    --merge 1 --merge-port 50110 --progress
