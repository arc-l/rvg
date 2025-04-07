#!/bin/bash

plannerType=$1
mapType=fixedStartAndGoal
difficulty=
size=Small
resolution=90
times=(3.605 2.98 3.693 2.7680000000000002 4.4270000000000005 0.8929999999999999 2.404 1.199 0.949 1.429)
for map_id in $(seq 0 9)
do
	mapPath=../Configs/map${mapType}${difficulty}${size}${map_id}.xml
	time=${times[$map_id]}
	echo "Running ${plannerType} on map${mapType}${difficulty}${size}${map_id}"
	../cmake-build-Release-vscode/mainOMPLConfigRun ${mapPath} $plannerType $time 10 >> "../Results/${plannerType}${mapType}${difficulty}${size}Map${map_id}_${resolution}.txt"
done
