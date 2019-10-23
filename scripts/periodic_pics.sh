#!/bin/bash

cnt = 0
while true; do
echo cnt: $cnt
raspistill -vf -w 800 -h 600 -o test$cnt.jpg
sleep 1
((cnt=cnt+1))
done
