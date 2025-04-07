#!/bin/bash

# check if the Results directory exists
if [ ! -d "../Results" ]; then
  mkdir ../Results
fi

for resolution in 8 18 36 72 90 180 360 ; do
  for plannerType in AITStar BITStar RRTStar; do
    # parallelize the runs
    ./runRVG_OMPL_RatioTest_${resolution}.sh $plannerType &
  done
done

echo "Done"