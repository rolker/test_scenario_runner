from results_loader import *
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages

if __name__ == "__main__":
    plt.style.use('seaborn-deep')
    pf_results = load_all_results("../pf_10_times/")
    planner_results = load_all_results("../short_10_times/")

    # print len(pf_results), len(planner_results)
    all_names = list(set(get_names(planner_results)))
    all_names.remove("ahead_short")
    all_names.sort()
    # counts = {r: all_names.count(r) for r in all_names}
    # print counts

    planner_completed_names = list(set(get_names(get_results_where(planner_results,
                                                                   lambda r: r["stats"]["uncovered_length"] == 0))))
    pf_completed_names = list(set(get_names(get_results_where(pf_results,
                                                              lambda r: r["stats"]["uncovered_length"] == 0))))
    planner_completed_names.remove("ahead_short")
    pf_completed_names.remove("ahead_short")

    planner_completed_names.sort()
    pf_completed_names.sort()

    planner_completed_counts = {r: planner_completed_names.count(r) for r in all_names}
    pf_completed_counts = {r: pf_completed_names.count(r) for r in all_names}

    # print planner_completed_counts
    # print pf_completed_counts

    planner_completed_counts_list = [planner_completed_counts[n] for n in all_names]
    pf_completed_counts_list = [pf_completed_counts[n] for n in all_names]

    planner_results = remove_not_finished(planner_results)
    pf_results = remove_not_finished(pf_results)

    # print len(planner_results)
    planner_results = get_results_where(planner_results, lambda r: r["name"] != "ahead_short")
    pf_results = get_results_where(pf_results, lambda r: r["name"] != "ahead_short")
    # print len(planner_results)

    # for l in [all_names, planner_completed_counts_list, pf_completed_counts_list]:
    #     print ''.join(map(str, [str(x) + " & " for x in l])), "\\\\"

    planner_by_name = [get_results_where(planner_results, lambda r: r["name"] == name) for name in pf_completed_names]
    pf_by_name = [get_results_where(pf_results, lambda r: r["name"] == name) for name in pf_completed_names]

    planner_median_scores = [np.median(np.array(get_scores(r))) for r in planner_by_name]
    planner_max_scores = [np.max(np.array(get_scores(r))) for r in planner_by_name]
    planner_min_scores = [np.min(np.array(get_scores(r))) for r in planner_by_name]

    # print planner_max_scores
    # print planner_median_scores
    # print planner_min_scores

    pf_median_scores = [np.median(np.array(get_scores(r))) for r in pf_by_name]
    pf_max_scores = [np.max(np.array(get_scores(r))) for r in pf_by_name]
    pf_min_scores = [np.min(np.array(get_scores(r))) for r in pf_by_name]

    # print "max", pf_max_scores
    # print "median", pf_median_scores
    # print "min", pf_min_scores

    # for name, max, median, min in zip(pf_completed_names, pf_max_scores, pf_median_scores, pf_min_scores):
    #     print name, max, median, min

    path = "../plots/pf_vs_planner/pf_vs_planner_score.pdf"
    title = "Potential Field vs Planner in Small Instances"
    y_label = "Median Score"
    x_label = "Test Name"
    label1 = "Planner"
    label2 = "Potential Field"
    # plot_side_by_side_bar_chart(path, planner_median_scores, pf_median_scores, pf_completed_names, title, y_label,
    #                             x_label, label1, label2, 'log')

    # get last plan achievable stats
    pf_achievable_count = sum([sum(r) for r in get_stats_item(pf_results, "last_plan_achievables")])
    total_iterations_pf = sum([len(r) for r in get_stats_item(pf_results, "last_plan_achievables")])
    print pf_achievable_count, total_iterations_pf, float(pf_achievable_count) / float(total_iterations_pf)
    planner_achievable_count = sum([sum(r) for r in get_stats_item(planner_results, "last_plan_achievables")])
    total_iterations_planner = sum([len(r) for r in get_stats_item(planner_results, "last_plan_achievables")])
    print planner_achievable_count, total_iterations_planner, float(planner_achievable_count) / float(total_iterations_planner)

    planner_achievable_fractions = np.array([get_achievable_fractions(r).mean() for r in planner_by_name]) * 100
    print planner_achievable_fractions

    pf_achievable_fractions = np.array([get_achievable_fractions(r).mean() for r in pf_by_name]) * 100
    print pf_achievable_fractions

    path = "../plots/pf_vs_planner/pf_vs_planner_achievable.pdf"
    title = "Potential Field vs Planner Achievable Plan Percentage"
    y_label = "Achievable Plans (%)"
    x_label = "Test Name"
    label1 = "Planner"
    label2 = "Potential Field"
    # plot_side_by_side_bar_chart(path, planner_achievable_fractions, pf_achievable_fractions, all_names, title, y_label,
    #                             x_label, label1, label2, 'linear')

    # okay now for long tests
    results = load_all_results("../pf_vs_planner_2")
    renaming = {
        "a": "b",
        "b": "c",
        "c": "d",
        "d": "e",
        "g": "a",
    }

    for result in results:
        for x, y in renaming.items():
            if x in result["name"]:
                result["name"] = result["name"].replace(x, y)
                break
    pf_results = get_results_with_specific_config_value(results, "planner_config", "use_potential_fields_planner", True)
    planner_results = get_results_with_specific_config_value(results, "planner_config", "use_potential_fields_planner",
                                                             False)
    # pf_results = load_all_results("../pf_long_tests/")
    # planner_results = load_all_results("../planner_long_tests/")

    print len(pf_results), len(planner_results)
    all_names = list(set(get_names(results)))
    all_names.sort()
    print all_names

    # remove run aground
    aground_names = get_names(get_results_where(pf_results, run_aground))
    print aground_names
    pf_results = remove_names(pf_results, aground_names)
    planner_results = remove_names(planner_results, aground_names)

    # sort by name
    pf_results.sort(key=by_name)
    planner_results.sort(key=by_name)

    # get scores
    pf_scores = get_scores(pf_results)
    planner_scores = get_scores(planner_results)

    print len(pf_scores), len(planner_scores)

    # generate plot
    path = "../plots/pf_vs_planner/pf_vs_planner_long_tests_score.pdf"
    title = "Potential Field vs Planner in Larger Instances"
    y_label = "Score"
    x_label = "Test Name"
    label1 = "Planner"
    label2 = "Potential Field"
    # plot_side_by_side_bar_chart(path, planner_scores, pf_scores, all_names, title, y_label, x_label, label1, label2,
    #                             'log')

    pepperrell_cove_results = load_all_results("../pepperrell_cove")
    print "Pepperrell Cove scores (planner, pf):", get_scores(pepperrell_cove_results)

    # table = plt.table(cellText=[planner_completed_counts_list, pf_completed_counts_list],
    #                   rowLabels=["Planner", "Potential Field"],
    #                   colLabels=all_names)
    #
    # table.set_fontsize(24)
    #
    # plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    # plt.tick_params(axis='y', which='both', right=False, left=False, labelleft=False)
    # for pos in ['right', 'top', 'bottom', 'left']:
    #     plt.gca().spines[pos].set_visible(False)

    # path = "../plots/pf_vs_planner/pf_vs_planner_completed_table_1.pdf"
    # with PdfPages(path) as pdf:
    #     # plt.rcParams["font.size"] = 20
    #
    #     fig = plt.figure()
    #     ax = fig.add_subplot(111)
    #     col_labels = all_names[:7]
    #     row_labels = ["Planner", "Potential Field"]
    #     table_vals = [planner_completed_counts_list[:7], pf_completed_counts_list[:7]]
    #
    #     # Draw table
    #     the_table = plt.table(cellText=table_vals,
    #                           # colWidths=[0.1] * len(col_labels),
    #                           rowLabels=row_labels,
    #                           colLabels=col_labels,
    #                           loc='center')
    #     the_table.auto_set_font_size(False)
    #     the_table.set_fontsize(5)
    #     the_table.scale(1, 1)
    #
    #     # Removing ticks and spines enables you to get the figure only with table
    #     plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    #     plt.tick_params(axis='y', which='both', right=False, left=False, labelleft=False)
    #     for pos in ['right','top','bottom','left']:
    #         plt.gca().spines[pos].set_visible(False)
    #
    #     pdf.savefig()
    #
    # path = "../plots/pf_vs_planner/pf_vs_planner_completed_table_2.pdf"
    # with PdfPages(path) as pdf:
    #     # plt.rcParams["font.size"] = 20
    #
    #     fig = plt.figure()
    #     ax = fig.add_subplot(111)
    #     col_labels = all_names[7:13]
    #     row_labels = ["Planner", "Potential Field"]
    #     table_vals = [planner_completed_counts_list[7:13], pf_completed_counts_list[7:13]]
    #
    #     # Draw table
    #     the_table = plt.table(cellText=table_vals,
    #                           # colWidths=[0.1] * len(col_labels),
    #                           rowLabels=row_labels,
    #                           colLabels=col_labels,
    #                           loc='center')
    #     the_table.auto_set_font_size(False)
    #     the_table.set_fontsize(5)
    #     the_table.scale(1, 1)
    #
    #     # Removing ticks and spines enables you to get the figure only with table
    #     plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    #     plt.tick_params(axis='y', which='both', right=False, left=False, labelleft=False)
    #     for pos in ['right','top','bottom','left']:
    #         plt.gca().spines[pos].set_visible(False)
    #
    #     pdf.savefig()
    #
    # path = "../plots/pf_vs_planner/pf_vs_planner_completed_table_3.pdf"
    # with PdfPages(path) as pdf:
    #     # plt.rcParams["font.size"] = 20
    #
    #     fig = plt.figure()
    #     ax = fig.add_subplot(111)
    #     col_labels = all_names[13:]
    #     row_labels = ["Planner", "Potential Field"]
    #     table_vals = [planner_completed_counts_list[13:], pf_completed_counts_list[13:]]
    #
    #     # Draw table
    #     the_table = plt.table(cellText=table_vals,
    #                           # colWidths=[0.1] * len(col_labels),
    #                           rowLabels=row_labels,
    #                           colLabels=col_labels,
    #                           loc='center')
    #     the_table.auto_set_font_size(False)
    #     the_table.set_fontsize(5)
    #     the_table.scale(1, 1)
    #
    #     # Removing ticks and spines enables you to get the figure only with table
    #     plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    #     plt.tick_params(axis='y', which='both', right=False, left=False, labelleft=False)
    #     for pos in ['right','top','bottom','left']:
    #         plt.gca().spines[pos].set_visible(False)
    #
    #     pdf.savefig()

