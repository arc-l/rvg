import matplotlib.pyplot as plt
import numpy as np
import os, sys

plt.rcParams['font.size'] = 30 
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']

if __name__ == "__main__":
    fig, axes = plt.subplots(2, 5, figsize=(15, 8), sharex=True, sharey=True)
    configs = list(range(10))
    resolutions = [8, 18, 36, 72, 90, 180, 360]
    map_type = "fixedStartAndGoal"
    difficulty = "Hard"
    size="Small"
    results_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/Results/"
    lines = []
    for config in configs:
        rvg_search_time = []
        rvg_build_time = []
        rvg_path_length = []
        for res in resolutions:
            log_path = os.path.join(results_path, f"RVG{map_type}{difficulty}{size}{config}_{res}.txt")
            print(f"Reading {log_path}")
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

        print(f"RVG Search Time for env{config}: {rvg_search_time}")
        print(f"RVG Build Time env{config}: {rvg_build_time}")
        print(f"RVG Path Length env{config}: {rvg_path_length}")

        # dual y-axis plot for time and path length
        ax1 = axes[config // 5, config % 5]
        ax1.set_yscale('log')
        ax2 = ax1.twinx()
        line1, = ax1.plot(resolutions, rvg_search_time, '*-', label="Search time", color='lightcoral', linewidth=3, markersize=10)
        line2, = ax1.plot(resolutions, rvg_build_time, '*-', label="Build time", color='olivedrab', linewidth=3, markersize=10)
        line3, = ax2.plot(resolutions, rvg_path_length, '*-', label="Path Length", color='royalblue', linewidth=3, markersize=10)
        # ax1.set_xlabel('Resolution')
        # ax1.set_ylabel('Build/Search Time (s)')
        # ax2.set_ylabel('Path Length')
        ax1.set_xticks([0, 180, 360])
        if not lines:
            lines = [line1, line2, line3]
            labels = [line.get_label() for line in lines]
    # plt.subplots_adjust(top=0.85, bottom=0.2)
    fig.tight_layout(pad=1.5)
    fig.subplots_adjust(hspace=0.15, wspace=0.8, bottom=0.2)

    fig.text(0.5, 0.1, 'Resolution', ha='center', va='center')
    fig.text(0.02, 0.5, 'Build/Search Time(s)', ha='center', va='center', rotation='vertical')
    fig.text(0.98, 0.5, 'Path Length', ha='center', va='center', rotation='vertical')

    fig.legend(lines, labels, loc='lower center', bbox_to_anchor=(0.5, -0.05), ncol=3)
    plt.savefig(f"{results_path}/RVG_Benchmark_Hard.png", dpi=100, bbox_inches='tight')
    # plt.savefig(f"{results_path}/RVG_Benchmark_Hard.pdf")
