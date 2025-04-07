#!/bin/bash

plannerType=$1
mapType=fixedStartAndGoal
difficulty=
size=Small
resolution=18
times=(0.278 0.228 0.277 0.22 0.316 0.08 0.187 0.099 0.08700000000000001 0.122)
for map_id in $(seq 0 9)
do
	mapPath=../Configs/map${mapType}${difficulty}${size}${map_id}.xml
	time=${times[$map_id]}
	echo "Running ${plannerType} on map${mapType}${difficulty}${size}${map_id}"
	../cmake-build-Release-vscode/mainOMPLConfigRun ${mapPath} $plannerType $time 10 >> "../Results/${plannerType}${mapType}${difficulty}${size}Map${map_id}_${resolution}.txt"
done
