import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import os, sys

# Update default rc settings
plt.rcParams['font.size'] = 16
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']

if __name__ == "__main__":
    configs = list(range(10))
    resolutions = [8, 18, 36, 72, 90, 180, 360]
    map_type = "fixedStartAndGoal"
    difficulty = ""
    size = "Small"
    results_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/Results/"
    search_time = []
    build_time = []
    path_length = []
    for config in configs:
        rvg_search_time = []
        rvg_build_time = []
        rvg_path_length = []
        for res in resolutions:
            log_path = os.path.join(results_path, f"RVG{map_type}{difficulty}{size}{config}_{res}.txt")
            with open(log_path, "r") as f:
                contents = f.readlines()
                for line_num in [-3, -2, -1]:
                    content = contents[line_num]
                    if "SearchTime" in content:
                        rvg_search_time.append(float(content.split(":")[-1]))
                    if "BuildTime" in content:
                        rvg_build_time.append(float(content.split(":")[-1]))
                    if "Path Length" in content:
                        rvg_path_length.append(float(content.split(":")[-1]))

        rvg_build_time = np.array(rvg_build_time)
        rvg_search_time = np.array(rvg_search_time)
        rvg_path_length = np.array(rvg_path_length)

        rvg_build_time = rvg_build_time / rvg_build_time[-1]
        rvg_search_time = rvg_search_time / rvg_search_time[-1]
        rvg_path_length = rvg_path_length / rvg_path_length[-1]

        # print(f"RVG Search Time: {rvg_search_time}")
        # print(f"RVG Build Time: {rvg_build_time}")
        # print(f"RVG Path Length: {rvg_path_length}")

        for i in range(len(resolutions)):
            search_time.append([config, resolutions[i], rvg_search_time[i]])
            build_time.append([config, resolutions[i], rvg_build_time[i]])
            path_length.append([config, resolutions[i], rvg_path_length[i]])

    df_search_time = pd.DataFrame(search_time, columns=['Config', 'Resolution', 'SearchTime'])
    df_build_time = pd.DataFrame(build_time, columns=['Config', 'Resolution', 'BuildTime'])
    df_path_length = pd.DataFrame(path_length, columns=['Config', 'Resolution', 'PathLength'])
    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    line1 = sns.lineplot(ax=ax1, data=df_build_time, x='Resolution', y='BuildTime', color='lightcoral', label='Build Time', legend=False)
    line2 = sns.lineplot(ax=ax1, data=df_search_time, x='Resolution', y='SearchTime', color='olivedrab', label='Search Time', legend=False)
    line3 = sns.lineplot(ax=ax2, data=df_path_length, x='Resolution', y='PathLength', color='royalblue', label='Path Length', legend=False)
    line1.set_label('Build Time Ratio')
    line2.set_label('Search Time Ratio')
    line3.set_label('Path Length Ratio')
    handles1, labels1 = ax1.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    ax1.set_ylabel('Build/Search Time Ratio')
    ax2.set_ylabel('Path Length Ratio')

    handles = handles1 + handles2
    labels = labels1 + labels2
    plt.tight_layout(rect=[0, 0.1, 1, 0.95])
    plt.subplots_adjust(bottom=0.3)
    plt.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=3)
    plt.savefig(f"{results_path}/RVG_Benchmark_relative.png", dpi=500)
    plt.show()
