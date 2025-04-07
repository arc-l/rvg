import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

results_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/Results/"

map_type = "fixedStartAndGoal"
difficulty = ""
size = "Small"

plt.rcParams['font.size'] = 30 
# Update default rc settings
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']


def main():
    resolutions = [8, 18, 36, 72, 90, 180, 360]
    results = []
    methods = {
        'RRTStar': 'RRT*',
        'AITStar': 'AIT*',
        'BITStar': 'BIT*',
        'PRMStar': 'PRM*',
        'LazyPRMStar': 'Lazy PRM*',
        'EITStar': 'EIT*',
        'LBTRRT': 'LBTRRT',
    }
    

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

            for method in methods:
                log_path = os.path.join(results_path,
                                        f"{method}{map_type}{difficulty}{size}Map{map_id}_{resolution}.txt")
                with open(log_path, "r") as f:
                    lines = f.readlines()
                    line = lines[-1]
                    distances = line.split("=")[-1]
                    distances = [*map(float, distances.split())]
                    print(distances)
                    for run_id in range(len(distances)):
                        distance = distances[run_id]
                        ratio = distance / rvg_path_length[-1]
                        if ratio < 1.0:
                            continue
                        results.append([methods[method], map_id, resolution, ratio, run_id])
            results.append(["RVG", map_id, resolution, 1.0, 0])

    df = pd.DataFrame(results, columns=["Method", "Map ID", "Resolution", "Ratio", "Run ID"])
    palette = ["#1f77b4", "#ff7f0e", "#2ca02c",  "#d62728", "#9467bd", "#8c564b", "#e377c2", "#7f7f7f"]
    color_palette = sns.color_palette(palette=palette)
    fig, ax = plt.subplots(figsize=(10, 5))
    sns.lineplot(x="Resolution", y="Ratio", data=df, hue="Method", ax=ax, palette=color_palette, linewidth=3)
    plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.set_yticks = [1.0, 1.1, 1.2, 1.3]
    # plt.tight_layout(rect=[0, 0.0, 1, 0.95])
    plt.savefig(f"{results_path}/ratio.png", bbox_inches='tight', dpi=100)
    # plt.show()


if __name__ == "__main__":
    main()
