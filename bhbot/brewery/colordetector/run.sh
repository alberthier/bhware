#!/bin/bash
# run.sh
# When this script exits, exit all background process also.
if ! [ -p input ] ; then
	mkfifo input
fi
trap 'kill $(jobs -p)' EXIT
tail -n +1 -f input | ./colordetector 2>err.txt 1>out.txt &
multitail err.txt out.txt
