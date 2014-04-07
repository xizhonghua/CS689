#!/bin/bash
rm *.log
for s in 1 2 3 4 5
do
    echo "Scene $s"
    for m in 1 2 3 4
    do
        echo "Method $m"
        logfile="m$m-$s.log"
        ./bin/Planner "Scene$s.txt" -m $m > $logfile &
    done
    wait
done
