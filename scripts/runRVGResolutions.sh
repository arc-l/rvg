#!/bin/bash

mapType="fixedStartAndGoal"
difficulty="Hard"
size="Small"
nThreads=1
i=$1
draw=0
# Loop over all gap widths
for resolution in 8 18 36 72 90 180 360 ; do
    echo "Running with RVG for map$i with resolution $resolution and difficulty $difficulty"
    # Execute the program with the current planner type and gap width
    if [ $draw -eq 1 ]; then
      ../cmake-build-Release-vscode/mainRVGConfigRun ../Configs/map$mapType$difficulty$size$i $resolution $nThreads ../Results/RVG${mapType}${difficulty}${size}${i}_${resolution}.png > ../Results/RVG${mapType}${difficulty}${size}${i}_${resolution}.txt
    else
      ../cmake-build-Release-vscode/mainRVGConfigRun ../Configs/map$mapType$difficulty$size$i.xml $resolution $nThreads > ../Results/RVG${mapType}${difficulty}${size}${i}_${resolution}.txt
    fi
done

echo "Done"