import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import os

results_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/Results/"

map_type = "randomStartAndGoal"
difficulty = "Hard"

plt.rcParams['font.size'] = 16
# Update default rc settings
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']


def main():
    results = []
    methods = {
        'RRTStar': 'RRT*',
        'BITStar': 'BIT*',
        'AITStar': 'AIT*',
    }

    times = [20]
    for method in methods:
        for map_id in range(10):
            for run_id in range(10):
                for time in times:
                    log_path = os.path.join(results_path,
                                            f"{method}{map_type}{difficulty}{time}Map{map_id}_{run_id}.txt")
                    with open(log_path, "r") as f:
                        lines = f.readlines()
                        line = lines[-1]
                        distance = float(line.split(":")[-1])
                        results.append([methods[method], map_id+1, run_id, distance])

    method = "RVG"
    rvg_search_time = []
    rvg_build_time = []
    rvg_path_length = []
    for map_id in range(10):
        log_path = os.path.join(results_path, f"{method}{map_type}{difficulty}Map{map_id}.txt")
        with open(log_path, "r") as f:
            lines = f.readlines()
            line = lines[-1]
            distance = float(line.split(":")[-1])
            results.append([method, map_id+1, 0, distance])
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

    fig, ax = plt.subplots(figsize=(15, 5))
    sns.barplot(x='MapID', y='Path Length', data=df, hue="Methods", ax=ax)
    # handles, labels = ax.get_legend_handles_labels()
    # fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.95), ncol=4)
    ytick_start = 20
    step_size = 5
    ytick_end = step_size * int((df.max()['Path Length'] / step_size))
    plt.yticks(np.arange(ytick_start, ytick_end + 1, step_size))
    plt.ylim(ytick_start, ytick_end)
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.4), ncol=4)
    plt.tight_layout(rect=[0, 0.0, 1, 1.0])
    plt.savefig(f"{results_path}/comparison_hard.pdf")
    # plt.show()


if __name__ == "__main__":
    main()
