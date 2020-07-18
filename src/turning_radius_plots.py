from results_loader import *
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    plt.style.use('seaborn-deep')
    short_radius_results = load_all_results("../short_radius/")
    default_results = load_all_results("../default_settings/")

    print len(short_radius_results), len(default_results)

    # contcat so we can check unfinished together
    results = short_radius_results + default_results
    print len(results)

    # check the unfinished and run aground tasks
    unfinished_names = [result["name"] for result in results if result["stats"]["uncovered_length"] > 0]
    print ("Unfinished tasks:", unfinished_names)
    # remove tasks where we ran aground (subset of unfinished) (none in this test set)
    run_aground_names = get_names(get_results_where(results, run_aground))
    print ("Run aground in tasks:", run_aground_names)
    # this time, also remove unfinished tasks
    short_radius_results = remove_names(short_radius_results, unfinished_names)
    default_results = remove_names(default_results, unfinished_names)

    # get names
    names = list({result["name"] for result in default_results})
    names.sort()
    print names

    # sort by name
    short_radius_results.sort(key=by_name)
    default_results.sort(key=by_name)

    # get scores
    short_radius_scores = get_scores(short_radius_results)
    default_scores = get_scores(default_results)

    # generate plot
    path = "../plots/short_turning_radius/short_turning_radius_score.pdf"
    title = "8m vs 5m Turning Radius"
    y_label = "Score"
    x_label = "Test Name"
    label1 = "8m"
    label2 = "5m"
    # plot_side_by_side_bar_chart(path, default_scores, short_radius_scores, names, title, y_label, x_label, label1, label2,
    #                             'linear')

    default_achievable_fractions = get_achievable_fractions(default_results) * 100
    print default_achievable_fractions

    short_achievable_fractions = get_achievable_fractions(short_radius_results) * 100
    print short_achievable_fractions

    path = "../plots/short_turning_radius/short_radius_achievable.pdf"
    title = "8m vs 5m Turning Radius Achievable Plan Percentage"
    y_label = "Achievable Plans (%)"
    x_label = "Test Name"
    label1 = "8m"
    label2 = "5m"
    plot_side_by_side_bar_chart(path, default_achievable_fractions, short_achievable_fractions, names, title, y_label,
                                x_label, label1, label2, 'linear')
