from results_loader import *
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    plt.style.use('seaborn-deep')
    gaussian_results = load_all_results("../gaussian_DO/")
    default_results = load_all_results("../default_settings/")

    print len(gaussian_results), len(default_results)

    # contcat so we can check unfinished together
    results = gaussian_results + default_results
    print len(results)

    # check the unfinished and run aground tasks
    unfinished_names = [result["name"] for result in results if result["stats"]["uncovered_length"] > 0]
    print ("Unfinished tasks:", unfinished_names)
    # remove tasks where we ran aground (subset of unfinished) (none in this test set)
    run_aground_names = get_names(get_results_where(results, run_aground))
    print ("Run aground in tasks:", run_aground_names)
    # this time, also remove unfinished tasks
    gaussian_results = remove_names(gaussian_results, unfinished_names)
    default_results = remove_names(default_results, unfinished_names)

    # remove tasks that don't have dynamic obstacles
    no_DO_names = ['adjacent_multi_line', 'adjacent_single_line', 'ahead_long', 'ahead_short', 'long_single_line',
                   'test01', 'test02a', 'test02b', 'test12a', 'test12b']
    gaussian_results = remove_names(gaussian_results, no_DO_names)
    default_results = remove_names(default_results, no_DO_names)

    # get names
    names = list({result["name"] for result in default_results})
    names.sort()
    print names

    # sort by name
    gaussian_results.sort(key=by_name)
    default_results.sort(key=by_name)

    # get scores
    gaussian_scores = get_scores(gaussian_results)
    default_scores = get_scores(default_results)

    # generate plot
    # path = "../plots/different_DO/different_DO_score.pdf"
    # title = "Gaussian Dynamic Obstacles vs Rectangle Dynamic Obstacles"
    # y_label = "Score"
    # x_label = "Test Name"
    # label1 = "Rectangle"
    # label2 = "Gaussian"
    # plot_side_by_side_bar_chart(path, default_scores, gaussian_scores, names, title, y_label, x_label, label1, label2,
    #                             'linear')

    gaussian_times = get_stats_item(gaussian_results, "total_time_from_planner")
    default_times = get_stats_item(default_results, "total_time_from_planner")

    # generate times plot
    # path = "../plots/different_DO/different_DO_time.pdf"
    # title = "Gaussian Dynamic Obstacles vs Rectangle Dynamic Obstacles"
    # y_label = "Completion Time"
    # x_label = "Test Name"
    # label1 = "Rectangle"
    # label2 = "Gaussian"
    # plot_side_by_side_bar_chart(path, default_times, gaussian_times, names, title, y_label, x_label, label1, label2,
    #                             'linear')

    collision_penalties = np.array(get_stats_item(gaussian_results, "cumulative_collision_penalty"))  # / gaussian_results[0]["planner_config"]["collision_checking_increment"]

    path = "../plots/different_DO/collision_penalty.pdf"
    title = "Gaussian Dynamic Obstacles Collision Penalty"
    y_label = "Collision Penalty"
    x_label = "Test Name"
    label1 = "Rectangle"
    label2 = "Gaussian"
    # plot_bar_chart(path, collision_penalties, names, title, y_label, x_label)
