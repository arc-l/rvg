#!/bin/bash

mapType="fixedStartAndGoal"
difficulty="Hard"
i=$1
draw=0
resolution=$2
# Loop over all gap widths
echo "Running with RVG for map$i with resolution $resolution and difficulty $difficulty"
# Execute the program with the current planner type and gap width
if [ $draw -eq 1 ]; then
    ../cmake-build-release/mainRVGConfigRun ../Configs/map$mapType$difficulty$i $resolution ../Results/RVGMap_${mapType}_${difficulty}_${i}_${resolution}.png > ../Results/RVGMap_${mapType}_${difficulty}_${i}_${resolution}.txt
else
    ../cmake-build-release/mainRVGConfigRun ../Configs/map$mapType$difficulty$i.xml $resolution > ../Results/RVGMap_${mapType}_${difficulty}_${i}_${resolution}.txt
fi

echo "Done"
