#!/bin/bash

mapType="fixedStartAndGoal"
difficulty=""
i=$1
draw=1
resolution=$2
threads=24
size="Small"
# Loop over all gap widths
echo "Running with RVG for map$i with resolution $resolution and difficulty $difficulty"
# Execute the program with the current planner type and gap width
if [ $draw -eq 1 ]; then
	../cmake-build-Release-vscode/mainRVGConfigRun ../Configs/map$mapType$difficulty$size$i.xml $resolution $threads ../Results/RVG$mapType$difficulty$size${i}_AllPathDemos.gif > "../Results/RVG${mapType}${difficulty}${size}${i}_AllPathDemos.txt"
else
    ../cmake-build-Release-vscode/mainRVGConfigRun ../Configs/map$mapType$difficulty$i.xml $resolution $threads > ../Results/RVGMap_${mapType}_${difficulty}_${i}_${resolution}.txt
fi

echo "Done"
