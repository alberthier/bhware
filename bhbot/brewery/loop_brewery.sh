#!/bin/sh

CURRENT_DIR=$(dirname $0)

cd ${CURRENT_DIR}

while true
do
    ./brewery.py

    if [[ "$?" != "72" ]] ; then
        break
    fi
done