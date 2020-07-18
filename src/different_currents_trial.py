from results_loader import *
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages


if __name__ == "__main__":
    plt.style.use('seaborn-deep')
    results = load_all_results("../different_currents_trial/")
    print len(results)
    unfinished_names = [result["name"] for result in results if result["stats"]["uncovered_length"] > 0]
    print ("Unfinished tasks:", unfinished_names)
    # remove these for now; study correlation later
    run_aground_names = get_names(get_results_where(results, run_aground))
    print ("Run aground in tasks:", run_aground_names)
    long_single_line_results = get_results_where(results, lambda r: r["name"] == "long_single_line")
    lsl_times = get_stats_item(long_single_line_results, "total_time_from_planner")
    print lsl_times  # OK, didn't run aground immediately. Should look into that
    lsl_run_aground = get_results_where(long_single_line_results, run_aground)
    print "Ran aground in", [r["path"] for r in lsl_run_aground]
    results = remove_names(results, run_aground_names)
    # I think this make a set, so it's unique
    current_values = list({result["sim_config"]["current_direction"] for result in results})
    current_values.sort()
    print current_values
    results_by_current = [get_results_with_specific_config_value(results, "sim_config", "current_direction", value)
                          for value in current_values]
    print len(results_by_current)
    print [len(r) for r in results_by_current]
    names = list({result["name"] for result in results})
    names.sort()
    print names
    results_by_name = [get_results_where(results, lambda r: r["name"] == name) for name in names]
    median_scores_by_name = np.array([np.median(np.array((get_scores(r)))) for r in results_by_name])
    print zip(names, median_scores_by_name)

    titles = ["Score differences from median with current pushing at " + str(current) + " degrees"
              for current in current_values]
    for r in results_by_current:
        r.sort(key=by_name)
    # TODO! -- compare with zero current trial
    # TODO! -- should consider counting number of times controller says we can't make reference trajectory
    scores_list = [np.array(get_scores(r)) - median_scores_by_name for r in results_by_current]
    path = '../plots/current_trial_1/dir_cost_difference_all.pdf'
    y_label = "Score Difference"
    x_label = "Test Name"
    # plot_several_bar_charts_to_one_pdf(path, scores_list, names, titles, y_label, x_label)
    # the same plots as above but individual pdfs
    for current, r in zip(current_values, results_by_current):
        r.sort(key=by_name)
        scores = np.array(get_scores(r)) - median_scores_by_name
        path = '../plots/current_trial_1/dir_' + str(current) + '_cost_difference.pdf'
        title = "Score differences from median with current pushing at " + str(current) + " degrees"
        y_label = "Score Difference"
        x_label = "Test Name"
        # plot_bar_chart(path, scores, names, title, y_label, x_label)

    for r in results_by_name:
        r.sort(key=lambda x: x["sim_config"]["current_direction"])
    scores_list = [get_scores(r) for r in results_by_name]
    path = '../plots/current_trial_1/cost_by_test.pdf'
    titles = ["Scores for " + name + " with different currents" for name in names]
    y_label = "Score"
    x_label = "Current Pushing at (degrees)"
    # plot_several_bar_charts_to_one_pdf(path, scores_list, current_values, titles, y_label, x_label)
    for r, name in zip(results_by_name, names):
        r.sort(key=lambda r: r["sim_config"]["current_direction"])
        path = '../plots/current_trial_1/' + name + '_cost.pdf'
        title = "Scores for " + name + " with different currents"
        y_label = "Score"
        x_label = "Current Pushing at (degrees)"
        scores = get_scores(r)
        # plot_bar_chart(path, scores, current_values, title, y_label, x_label)


