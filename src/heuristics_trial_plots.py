from results_loader import *
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    plt.style.use('seaborn-deep')
    results = load_all_results("../heuristic_trial/")

    print len(results)

    results.sort(key=lambda r: r["stats"]["score"])

    heuristic_names = {
        0: "Euclidean TSP",
        1: "Euclidean K-TSP",
        2: "Sum",
        3: "Dubins TSP",
        4: "Dubins K-TSP"
    }

    scores = get_scores(results)
    names = [heuristic_names[h] for h in [r["planner_config"]["heuristic"] for r in results]]

    print zip(names, scores)

    # generate plot
    path = "../plots/heuristics_trial/heuristics_score.pdf"
    title = "Varying Heuristics in adjacent_multi_line Test"
    y_label = "Score"
    x_label = "Heuristic"
    plot_bar_chart(path, scores, names, title, y_label, x_label)
