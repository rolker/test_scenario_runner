from results_loader import *
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    plt.style.use('seaborn-deep')
    results = load_all_results("../long_horizon/")
    print len(results)

    # check the unfinished and run aground tasks
    unfinished_names = [result["name"] for result in results if result["stats"]["uncovered_length"] > 0]
    print ("Unfinished tasks:", unfinished_names)
    # remove tasks where we ran aground (subset of unfinished)
    run_aground_names = get_names(get_results_where(results, run_aground))
    print ("Run aground in tasks:", run_aground_names)
    results = remove_names(results, run_aground_names)
    # this time, also remove unfinished tasks
    results = remove_names(results, unfinished_names)

    # get names
    names = list({result["name"] for result in results})
    names.sort()
    print names

    # split into the two groups
    long_horizon_results = get_results_with_specific_config_value(results, "planner_config", "time_horizon", 60)
    print len(long_horizon_results)

    normal_horizon_results = load_all_results("../default_settings")
    normal_horizon_results = remove_names(normal_horizon_results, unfinished_names)
    normal_horizon_results = remove_names(normal_horizon_results, run_aground_names)
    print len(normal_horizon_results)

    # sort by name
    long_horizon_results.sort(key=by_name)
    normal_horizon_results.sort(key=by_name)

    # get scores
    lh_scores = get_scores(long_horizon_results)
    nh_scores = get_scores(normal_horizon_results)

    # generate plot
    path = "../plots/long_horizon/long_horizon_scores.pdf"
    title = "60s Horizon vs 30s Horizon"
    y_label = "Score"
    x_label = "Test Name"
    label1 = "30s Horizon"
    label2 = "60s Horizon"
    # plot_side_by_side_bar_chart(path, nh_scores, lh_scores, names, title, y_label, x_label, label1, label2, "linear")


