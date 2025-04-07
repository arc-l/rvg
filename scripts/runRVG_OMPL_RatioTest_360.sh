#!/bin/bash

plannerType=$1
mapType=fixedStartAndGoal
difficulty=
size=Small
resolution=360
times=(55.59 46.328 53.446000000000005 38.966 63.725 11.939 33.805 17.317999999999998 12.554 19.652)
for map_id in $(seq 0 9)
do
	mapPath=../Configs/map${mapType}${difficulty}${size}${map_id}.xml
	time=${times[$map_id]}
	echo "Running ${plannerType} on map${mapType}${difficulty}${size}${map_id}"
	../cmake-build-Release-vscode/mainOMPLConfigRun ${mapPath} $plannerType $time 10 >> "../Results/${plannerType}${mapType}${difficulty}${size}Map${map_id}_${resolution}.txt"
done
