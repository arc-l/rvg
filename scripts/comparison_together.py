import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import os

results_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/Results/"

map_type = "randomStartAndGoal"
difficulty = ""

plt.rcParams['font.size'] = 30
# Update default rc settings
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']

size="Small"
methods = {
    'RRTStar': 'RRT*',
    'AITStar': 'AIT*',
    'BITStar': 'BIT*',
    'PRMStar': 'PRM*',
    'LazyPRMStar': 'Lazy PRM*',
    'EITStar': 'EIT*',
    'LBTRRT': 'LBTRRT',
}

def easy():

    times = [1, 10]
    results = []
    max_dist = []
    for method in methods:
        for map_id in range(10):
            for run_id in range(10):
                for time in times:
                    log_path = os.path.join(results_path,
                                            f"{method}{map_type}{difficulty}{time}{size}Map{map_id}_{run_id}.txt")
                    with open(log_path, "r") as f:
                        lines = f.readlines()
                        line = lines[-2]
                        distance = float(line.split(":")[-1])
                        max_dist.append(distance)
                        results.append([methods[method] + f"({time}s)", map_id + 1, run_id, distance])

    method = "RVG"
    rvg_search_time = []
    rvg_build_time = []
    rvg_path_length = []
    for map_id in range(10):
        log_path = os.path.join(results_path, f"{method}{map_type}{difficulty}{size}{map_id}_36.txt")
        with open(log_path, "r") as f:
            lines = f.readlines()
            line = lines[-1]
            distance = float(line.split(":")[-1])
            results.append([method, map_id + 1, 0, distance])
            for line_num in [-3, -2, -1]:
                content = lines[line_num]
                if "SearchTime" in content:
                    rvg_search_time.append(float(content.split(":")[-1]))
                if "BuildTime" in content:
                    rvg_build_time.append(float(content.split(":")[-1]))
                if "Path Length" in content:
                    rvg_path_length.append(float(content.split(":")[-1]))

    rvg_path_length = np.array(rvg_path_length)
    rvg_search_time = np.array(rvg_search_time)
    rvg_build_time = np.array(rvg_build_time)
    total_time = rvg_search_time + rvg_build_time
    mean_time = np.mean(total_time)
    print(mean_time)

    df = pd.DataFrame(results, columns=['Methods', 'MapID', 'RunID', 'Path Length'])
    df.replace("RVG", f"RVG", inplace=True)
    return df, max(max_dist)


def hard():
    map_type = "randomStartAndGoal"
    difficulty = "Hard"
    results = []

    times = [20]
    max_dist = []
    for method in methods:
        for map_id in range(10):
            for run_id in range(10):
                for time in times:
                    log_path = os.path.join(results_path,
                                            f"{method}{map_type}{difficulty}{time}{size}Map{map_id}_{run_id}.txt")
                    with open(log_path, "r") as f:
                        lines = f.readlines()
                        line = lines[-2]
                        distance = float(line.split(":")[-1])
                        max_dist.append(distance)
                        results.append([methods[method], map_id + 1, run_id, distance])

    method = "RVG"
    rvg_search_time = []
    rvg_build_time = []
    rvg_path_length = []
    for map_id in range(10):
        log_path = os.path.join(results_path, f"{method}{map_type}{difficulty}{size}{map_id}_36.txt")
        with open(log_path, "r") as f:
            lines = f.readlines()
            line = lines[-1]
            distance = float(line.split(":")[-1])
            results.append([method, map_id + 1, 0, distance])
            for line_num in [-3, -2, -1]:
                content = lines[line_num]
                if "SearchTime" in content:
                    rvg_search_time.append(float(content.split(":")[-1]))
                if "BuildTime" in content:
                    rvg_build_time.append(float(content.split(":")[-1]))
                if "Path Length" in content:
                    rvg_path_length.append(float(content.split(":")[-1]))

    rvg_path_length = np.array(rvg_path_length)
    rvg_search_time = np.array(rvg_search_time)
    rvg_build_time = np.array(rvg_build_time)
    total_time = rvg_search_time + rvg_build_time
    mean_time = np.mean(total_time)
    print(mean_time)

    df = pd.DataFrame(results, columns=['Methods', 'MapID', 'RunID', 'Path Length'])
    df.replace("RVG", f"RVG", inplace=True)
    return df, max(max_dist)


def main():
    df_easy, max_dist_easy = easy()
    df_hard, max_dist_hard = hard()
    dfs = [df_easy, df_hard]
    max_dists = [max_dist_easy, max_dist_hard]
    fig, axes = plt.subplots(2, 1, figsize=(30, 8), sharex=True, sharey=False)
    
    # Define colors for each method
    # color_palette = sns.color_palette("tab10", len(dfs[0]['Methods'].unique()))
    palette = ["#1f77b4", "#1f77b4", "#ff7f0e",  "#ff7f0e", "#2ca02c",  "#2ca02c", "#d62728",  "#d62728", "#9467bd",  "#9467bd", "#8c564b",  "#8c564b", "#e377c2",  "#e377c2", "#7f7f7f"]
    color_palette_easy = sns.color_palette(palette=palette)
    palette_hard = ["#1f77b4", "#ff7f0e", "#2ca02c",  "#d62728", "#9467bd", "#8c564b", "#e377c2", "#7f7f7f"]
    color_palette_hard = sns.color_palette(palette=palette_hard)
    color_palettes = [color_palette_easy, color_palette_hard]
    

    hatches = ["x", "", "*", "x", "+", "|"]
    ytick_ends = [50, 60]

    for i, df in enumerate(dfs):
        bars = sns.barplot(x='MapID', y='Path Length', data=df, hue="Methods", ax=axes[i], palette=color_palettes[i])

        if i == 0:  # Apply hatches for the "easy" plot
            # Assign hatches to the bars based on time (1s or 10s)
            for j, patch in enumerate(bars.patches):
                # patch.set_hatch(hatches[j // (2 * len(methods)+1) % 2])
                patch.set_hatch(hatches[j // 10 % 2])

        for j, patch in enumerate(bars.patches):
            patch.set_edgecolor('black')
        ytick_start = 10 
        # ytick_end = max_dists[i] + 10
        ytick_end = ytick_ends[i]
        axes[i].set_yticks(np.arange(ytick_start, ytick_end+1, 10))
        axes[i].set_ylim(ytick_start, ytick_end)
        
        # Put the legend on the right side
        axes[i].legend(loc='center left', bbox_to_anchor=(1, 0.5), ncol=1)
    
    # Add a combined legend below the plots
    handles, labels = axes[1].get_legend_handles_labels()  # Get legend information from the first plot
    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.07), ncol=len(handles))
    
    # Remove individual legends
    axes[0].legend_.remove()
    axes[1].legend_.remove()

    
    plt.tight_layout(rect=[0, 0.0, 1, 1])
    plt.savefig(f"{results_path}/comparison_together.png", dpi=300, bbox_inches='tight')
    # plt.show()


if __name__ == "__main__":
    main()
