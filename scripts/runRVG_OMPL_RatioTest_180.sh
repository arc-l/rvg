#!/bin/bash

plannerType=$1
mapType=fixedStartAndGoal
difficulty=
size=Small
resolution=180
times=(13.67 11.289 13.403 9.802 15.914000000000001 3.189 8.488 4.33 3.31 4.99)
for map_id in $(seq 0 9)
do
	mapPath=../Configs/map${mapType}${difficulty}${size}${map_id}.xml
	time=${times[$map_id]}
	echo "Running ${plannerType} on map${mapType}${difficulty}${size}${map_id}"
	../cmake-build-Release-vscode/mainOMPLConfigRun ${mapPath} $plannerType $time 10 >> "../Results/${plannerType}${mapType}${difficulty}${size}Map${map_id}_${resolution}.txt"
done
