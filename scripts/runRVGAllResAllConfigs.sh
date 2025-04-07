#!/bin/bash

mapType="randomStartAndGoal"
difficulty="Hard"
size="Small"
draw=0
nThreads=1

# check if Results directory exists
if [ ! -d "../Results" ]; then
  mkdir ../Results
fi

# Loop over all gap widths
for resolution in 8 18 36 72 90 180 360 ; do
  for i in {0..9}; do
    echo "Running with RVG for map$i with resolution $resolution and difficulty $difficulty"
    # Execute the program with the current planner type and gap width
    if [ $draw -eq 1 ]; then
      ../cmake-build-Release-vscode/mainRVGConfigRun ../Configs/map$mapType$difficulty$size$i.xml $resolution $nThreads ../Results/RVG${mapType}${difficulty}${i}_${resolution}.png > ../Results/RVG${mapType}${difficulty}{size}${i}_${resolution}.txt
    else
      ../cmake-build-Release-vscode/mainRVGConfigRun ../Configs/map$mapType$difficulty$size$i.xml $resolution $nThreads > ../Results/RVG${mapType}${difficulty}${size}${i}_${resolution}.txt
    fi
  done
done

echo "Done"
