#!/bin/bash


mapType="fixedStartAndGoal"
difficulty=""
size="Small"

for i in {0..9} 
do
    ../cmake-build-Release-vscode/mainDrawMap ../Configs/map$mapType$difficulty$size$i.xml ../Results/map$mapType$difficulty$size$i.png
done 


echo "Done"