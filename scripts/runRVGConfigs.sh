#!/bin/bash

# check if the Results directory exists
if [ ! -d "../Results" ]; then
  mkdir ../Results
fi

mapType="fixedStartAndGoal"
difficulty="Hard"
size="Small"
resolution=36
threads=24
# Loop over all gap widths
for i in {0..9}; do
	echo "Running with RVG for map$i $mapType $difficulty $size for resolution $resolution"
	../cmake-build-Release-vscode/mainRVGConfigRun ../Configs/map$mapType$difficulty$size$i.xml $resolution $threads ../Results/RVG$mapType$difficulty$size${i}_AllPathDemos.gif > "../Results/RVG${mapType}${difficulty}${size}${i}_AllPathDemos.txt"
done

echo "Done"
