from results_loader import *
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages


def plotPDF(path, x1, x2, names, title, y_label, label1, label2):
    with PdfPages(path) as pdf:
        # change the size (not necessary) units are in inches
        # plt.figure(figsize=(10, 5))

        # change font size (make test readable)
        plt.rcParams["font.size"] = 8
        # force labels to be visible
        fig, ax = plt.subplots(tight_layout=True)

        # plot data...
        N = len(x1)
        ind = np.arange(N)  # the x locations for the groups
        width = 0.27       # the width of the bars

        # TODO! -- log scale for Y axis
        rects1 = ax.bar(ind, x2, width, color='r', label=label2)
        rects2 = ax.bar(ind + width, x1, width, color='b', label=label1)

        ax.set_ylabel(y_label)
        ax.set_yscale('log')
        ax.set_xticks(ind + width/2)
        ax.set_xticklabels(names)
        # rotate names so you can read them
        plt.xticks(rotation=90)

        ax.legend(loc='upper right')
        # plt.show()

        plt.title(title)
        pdf.savefig()  # saves the current figure into a pdf page
        plt.close()


def plot_side_by_side_bar_chart(path, x1, x2, names, title, y_label, label1, label2):
    with PdfPages(path) as pdf:
        # change the size (not necessary) units are in inches
        # plt.figure(figsize=(10, 5))

        # change font size (make test readable)
        plt.rcParams["font.size"] = 8
        # force labels to be visible
        fig, ax = plt.subplots(tight_layout=True)

        # plot data...
        N = len(x1)
        ind = np.arange(N)  # the x locations for the groups
        width = 0.27       # the width of the bars

        rects1 = ax.bar(ind, x2, width, color='r', label=label2)
        rects2 = ax.bar(ind + width, x1, width, color='b', label=label1)

        ax.set_ylabel(y_label)
        ax.set_yscale('log')
        ax.set_xticks(ind + width/2)
        ax.set_xticklabels(names)
        # rotate names so you can read them
        plt.xticks(rotation=90)

        ax.legend(loc='upper right')
        # plt.show()

        plt.title(title)
        pdf.savefig()  # saves the current figure into a pdf page
        plt.close()


if __name__ == "__main__":
    plt.style.use('seaborn-deep')
    results = load_all_results("../pf_vs_planner_1/")
    print len(results)
    # should really use some sort of filter for most of these...
    pf_results = [result for result in results if result["planner_config"]["use_potential_fields_planner"]]
    print len(pf_results)
    planner_results = [result for result in results if not result["planner_config"]["use_potential_fields_planner"] and
                       result["planner_config"]["slow_speed"] == 1]
    print len(planner_results)
    adjacent_single_line_pf_results = [result for result in pf_results if result["name"] == "adjacent_single_line"]
    print adjacent_single_line_pf_results[0]["stats"]
    planner_uncovered_remaining = sum([result["stats"]["uncovered_length"] for result in planner_results])
    pf_uncovered_remaining = sum([result["stats"]["uncovered_length"] for result in pf_results])
    print ("Uncovered remaining with slowing allowed:", planner_uncovered_remaining)
    print ("Uncovered remaining with no slowing allowed:", pf_uncovered_remaining)
    unfinished_names = [result["name"] for result in results if result["stats"]["uncovered_length"] > 0]
    print ("Unfinished tasks:", unfinished_names)
    pf_results = remove_names(pf_results, unfinished_names)
    planner_results = remove_names(planner_results, unfinished_names)
    planner_names = [result["name"] for result in planner_results]
    # not including a couple of results from the planner so gotta chuck em here
    pf_results = [result for result in pf_results if result["name"] in planner_names]
    pf_results.sort(key=by_name)
    planner_results.sort(key=by_name)
    print len(pf_results)
    print len(planner_results)
    planner_times = [result["stats"]["total_time_from_planner"] for result in planner_results]
    pf_times = [result["stats"]["total_time_from_planner"] for result in pf_results]
    planner_scores = [result["stats"]["score"] for result in planner_results]
    pf_scores = [result["stats"]["score"] for result in pf_results]
    print planner_times
    print pf_times
    time_differences = [a - b for a, b in zip(planner_times, pf_times)]
    print time_differences

    names = [result["name"] for result in planner_results]

    plotPDF('../plots/pf_vs_planner_1_time.pdf', planner_times, pf_times, names,
            "Potential Fields vs Planner completion times", 'Completion Time', "Planner", "Potential Fields")
    plotPDF('../plots/pf_vs_planner_1_score.pdf', planner_scores, pf_scores, names,
            "Potential Fields vs Planner costs", 'Cost', "Planner", "Potential Fields")
