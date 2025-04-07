import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import os, sys

# Update default rc settings
plt.rcParams['font.size'] = 30 
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']

configs = list(range(10))
resolutions = [8, 18, 36, 72, 90, 180, 360]
map_type = "fixedStartAndGoal"
size = "Small"
results_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/Results/"

def easy():
    difficulty = ""
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

        for i in range(len(resolutions)):
            search_time.append([config, resolutions[i], rvg_search_time[i]])
            build_time.append([config, resolutions[i], rvg_build_time[i]])
            path_length.append([config, resolutions[i], rvg_path_length[i]])

    df_search_time = pd.DataFrame(search_time, columns=['Config', 'Resolution', 'Search Time Ratio'])
    df_build_time = pd.DataFrame(build_time, columns=['Config', 'Resolution', 'Build Time Ratio'])
    df_path_length = pd.DataFrame(path_length, columns=['Config', 'Resolution', 'Path Length Ratio'])
    # ax2 = ax1.twinx()
    # line1 = sns.lineplot(ax=ax1, data=df_build_time, x='Resolution', y='BuildTime', color='lightcoral', label='Build Time', legend=False)
    # line2 = sns.lineplot(ax=ax1, data=df_search_time, x='Resolution', y='SearchTime', color='olivedrab', label='Search Time', legend=False)
    # line3 = sns.lineplot(ax=ax2, data=df_path_length, x='Resolution', y='PathLength', color='royalblue', label='Path Length', legend=False)
    # line1.set_label('Build Time Ratio')
    # line2.set_label('Search Time Ratio')
    # line3.set_label('Path Length Ratio')
    # handles1, labels1 = ax1.get_legend_handles_labels()
    # handles2, labels2 = ax2.get_legend_handles_labels()
    # ax1.set_ylabel('Build/Search Time Ratio')
    # ax2.set_ylabel('Path Length Ratio')

    # handles = handles1 + handles2
    # labels = labels1 + labels2
    # plt.tight_layout(rect=[0, 0.1, 1, 0.95])
    # plt.subplots_adjust(bottom=0.3)
    # plt.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=3)
    # plt.savefig(f"{results_path}/RVG_Benchmark_relative.png", dpi=500)
    # plt.show()
    return df_search_time, df_build_time, df_path_length

def difficult():
    search_time = []
    build_time = []
    path_length = []
    difficulty = "Hard"
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

        for i in range(len(resolutions)):
            search_time.append([config, resolutions[i], rvg_search_time[i]])
            build_time.append([config, resolutions[i], rvg_build_time[i]])
            path_length.append([config, resolutions[i], rvg_path_length[i]])

    df_search_time = pd.DataFrame(search_time, columns=['Config', 'Resolution', 'Search Time Ratio'])
    df_build_time = pd.DataFrame(build_time, columns=['Config', 'Resolution', 'Build Time Ratio'])
    df_path_length = pd.DataFrame(path_length, columns=['Config', 'Resolution', 'Path Length Ratio'])
    return df_search_time, df_build_time, df_path_length


if __name__ == "__main__":
    df_search_time_easy, df_build_time_easy, df_path_length_easy = easy()
    df_search_time_hard, df_build_time_hard, df_path_length_hard = difficult()
    df_search_times = [df_search_time_easy, df_search_time_hard]
    df_build_times = [df_build_time_easy, df_build_time_hard]
    df_path_lengths = [df_path_length_easy, df_path_length_hard]

    fig, axes = plt.subplots(1, 2, figsize=(15, 8), sharex=True, sharey=True)
    for i in range(2):
        ax1 = axes[i]
        for spine in ax1.spines.values():
            spine.set_linewidth(2)  # Change 2 to any value for desired thickness

        ax2 = ax1.twinx()
        line1 = sns.lineplot(ax=ax1, data=df_build_times[i], x='Resolution', y='Build Time Ratio', color='lightcoral', label='Build Time Ratio', legend=False, linewidth=3.0, markersize=3.0)
        line2 = sns.lineplot(ax=ax1, data=df_search_times[i], x='Resolution', y='Search Time Ratio', color='olivedrab', label='Search Time Ratio', legend=False, linewidth=3.0, markersize=3.0)
        line3 = sns.lineplot(ax=ax2, data=df_path_lengths[i], x='Resolution', y='Path Length Ratio', color='royalblue', label='Path Length Ratio', legend=False, linewidth=3.0, markersize=3.0)
        line1.set(xlabel=None)
        line2.set(xlabel=None)
        line3.set(xlabel=None)
        line1.set(ylabel=None)
        line2.set(ylabel=None)
        line3.set(ylabel=None)
        handles1, labels1 = ax1.get_legend_handles_labels()
        handles2, labels2 = ax2.get_legend_handles_labels()
        # ax1.set_ylabel('Build/Search Time Ratio')
        # ax2.set_ylabel('Path Length Ratio')
# 
        handles = handles1 + handles2
        labels = labels1 + labels2
    
    fig.tight_layout(pad=1.5)
    fig.subplots_adjust(hspace=0.15, wspace=0.8, bottom=0.2)

    fig.text(0.5, 0.2, 'Resolution', ha='center', va='center')
    fig.text(0.02, 0.6, 'Build/Search Time Ratio', ha='center', va='center', rotation='vertical')
    fig.text(0.98, 0.6, 'Path Length Ratio', ha='center', va='center', rotation='vertical')

    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, 0.05), ncol=3)
    plt.tight_layout(rect=[0.02, 0.1, 0.98, 0.95])
    plt.subplots_adjust(bottom=0.3)
    plt.savefig(f"{results_path}/RVG_Benchmark_relative_together.png", dpi=100, bbox_inches='tight')
