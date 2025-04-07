import os
import numpy as np

results_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/Results/"

map_type = "fixedStartAndGoal"
difficulty = ""
size = "Small"


def main():
    resolutions = [8, 18, 36, 72, 90, 180, 360]
    for resolution in resolutions:
        map_paths = []
        rvg_search_time = []
        rvg_build_time = []
        rvg_path_length = []
        for map_id in range(10):
            log_path = os.path.join(results_path, f"RVG{map_type}{difficulty}{size}{map_id}_{resolution}.txt")
            map_paths.append(f"map{map_type}{difficulty}{map_id}.xml")
            with open(log_path, "r") as f:
                lines = f.readlines()
                for line_num in [-3, -2, -1]:
                    content = lines[line_num]
                    if "SearchTime" in content:
                        rvg_search_time.append(float(content.split(":")[-1]))
                    if "BuildTime" in content:
                        rvg_build_time.append(float(content.split(":")[-1]))
                    if "Path Length" in content:
                        rvg_path_length.append(float(content.split(":")[-1]))

        print(map_paths)
        rvg_build_time = np.array(rvg_build_time)
        rvg_search_time = np.array(rvg_search_time)
        rvg_path_length = np.array(rvg_path_length)
        total_time = rvg_build_time + rvg_search_time
        print("Total time: ", total_time)
        print("Path length: ", rvg_path_length)

        path_name = f"runRVG_OMPL_RatioTest_{resolution}.sh"
        bash_script = f"#!/bin/bash\n\n"
        bash_script += f"plannerType=$1\n"
        bash_script += f"mapType={map_type}\n"
        bash_script += f"difficulty={difficulty}\n"
        bash_script += f"size={size}\n"
        bash_script += f"resolution={resolution}\n"
        bash_script += f"times=({total_time[0]} {total_time[1]} {total_time[2]} {total_time[3]} {total_time[4]} {total_time[5]} {total_time[6]} {total_time[7]} {total_time[8]} {total_time[9]})\n"
        bash_script += f"for map_id in $(seq 0 9)\n"
        bash_script += f"do\n"
        bash_script += f"\tmapPath=../Configs/map${{mapType}}${{difficulty}}${{size}}${{map_id}}.xml\n"
        bash_script += f"\ttime=${{times[$map_id]}}\n"
        bash_script += f'\techo "Running ${{plannerType}} on map${{mapType}}${{difficulty}}${{size}}${{map_id}}"\n'
        bash_script += '\t../cmake-build-Release-vscode/mainOMPLConfigRun ${mapPath} $plannerType $time 10 >> "../Results/${plannerType}${mapType}${difficulty}${size}Map${map_id}_${resolution}.txt"\n'
        bash_script += f"done\n"

        with open(path_name, "w") as f:
            f.write(bash_script)


if __name__ == "__main__":
    main()
