#!/bin/bash

# check if the Results directory exists
if [ ! -d "../Results" ]; then
  mkdir ../Results
fi

plannerType=$1
mapType="randomStartAndGoal"
difficulty="Hard"
size="Small"
time=20
# Loop over all gap widths
 for i in {0..9}; do
   for map in {0..9}; do
    echo "Running with $plannerType for map$map $mapType $difficulty for time $i"
    # Execute the program with the current planner type and gap width
    ../cmake-build-Release-vscode/mainOMPLConfigRun ../Configs/map$mapType$difficulty$size$map.xml $plannerType $time >> "../Results/${plannerType}${mapType}${difficulty}${time}${size}Map${map}_${i}.txt"
  done
done

echo "Done"