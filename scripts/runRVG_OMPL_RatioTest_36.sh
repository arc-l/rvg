#!/bin/bash

plannerType=$1
mapType=fixedStartAndGoal
difficulty=
size=Small
resolution=36
times=(0.759 0.619 0.7190000000000001 0.556 0.877 0.196 0.47400000000000003 0.254 0.21000000000000002 0.307)
for map_id in $(seq 0 9)
do
	mapPath=../Configs/map${mapType}${difficulty}${size}${map_id}.xml
	time=${times[$map_id]}
	echo "Running ${plannerType} on map${mapType}${difficulty}${size}${map_id}"
	../cmake-build-Release-vscode/mainOMPLConfigRun ${mapPath} $plannerType $time 10 >> "../Results/${plannerType}${mapType}${difficulty}${size}Map${map_id}_${resolution}.txt"
done
