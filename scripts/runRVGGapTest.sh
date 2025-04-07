#!/bin/bash

# Array of gap widths
# gapWidths=( 1.01 1.02 1.05 1.1 1.2 1.5 1.75 2.0 )
gapWidths=(2.0)

# Loop over all gap widths
for gap in "${gapWidths[@]}"; do
	echo "Running with planner type $planner and gap width $gap"
	# Execute the program with the current planner type and gap width
	../cmake-build-release/mainRVG_OMPL_ComparisonNarrowPassage RVG "$gap"
done

echo "All simulations have been executed."
