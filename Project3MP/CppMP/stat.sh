#!/bin/bash
for s in 1 2 3 4 5
do
    for m in 1 2 3 4
    do
        logfile="m$m-$s.log"
        tmp=$logfile.tmp
        
        #count solved        
        solved=`cat $logfile | grep "Solved = 1" | wc -l`
        
        # stats time
        cat $logfile | grep "Solved = " | cut -f3 -d ' ' | stats > $tmp
        mean_t=`cat $tmp | grep "mean" | cut -f14 -d ' '`
        stddev_t=`cat $tmp | grep "std" | cut -f12 -d ' '`
        
        # stats node
        cat $logfile | grep "Solved = " | cut -f12 -d ' ' | stats > $tmp
        mean_n=`cat $tmp | grep "mean" | cut -f14 -d ' '`
        stddev_n=`cat $tmp | grep "std" | cut -f12 -d ' '`
        
        rm $tmp
        echo $s $m $solved $mean_t $stddev_t $mean_n $stddev_n
    done
    wait
done
