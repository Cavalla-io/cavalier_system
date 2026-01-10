#!/bin/bash

if timeout 2 candump can0 -n 1 > /dev/null 2>&1; then
    echo "CAN bus has traffic"
else
    echo "No CAN traffic detected"
fi
