from results_loader import *
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages


def plotPDF(path, x1, x2, names, title):
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

        rects1 = ax.bar(ind, x2, width, color='r', label="slowing not allowed")
        rects2 = ax.bar(ind + width, x1, width, color='b', label="slowing allowed")

        ax.set_ylabel('Score')
        # ax.set_yscale('log')
        ax.set_xticks(ind + width/2)
        ax.set_xticklabels(names)
        # rotate names so you can read them
        plt.xticks(rotation=90)

        ax.legend(loc='upper center')
        # plt.show()

        plt.title(title)
        pdf.savefig()  # saves the current figure into a pdf page
        plt.close()


if __name__ == "__main__":
    plt.style.use('seaborn-deep')
    results = load_all_results("../slow_no_slow_1/")
    print len(results)
    # should really use some sort of filter for most of these...
    no_slow_results = [result for result in results if result["planner_config"]["slow_speed"] == 0]
    print len(no_slow_results)
    slow_results = load_all_results("../default_settings/")
    print len(slow_results)
    slow_uncovered_remaining = sum([result["stats"]["uncovered_length"] for result in slow_results])
    no_slow_uncovered_remaining = sum([result["stats"]["uncovered_length"] for result in no_slow_results])
    print ("Uncovered remaining with slowing allowed:", slow_uncovered_remaining)
    print ("Uncovered remaining with no slowing allowed:", no_slow_uncovered_remaining)
    unfinished_names = [result["name"] for result in results if result["stats"]["uncovered_length"] > 0]
    print ("Unfinished tasks:", unfinished_names)
    no_slow_results = [result for result in no_slow_results if result["name"] not in unfinished_names]
    slow_results = [result for result in slow_results if result["name"] not in unfinished_names]
    no_slow_names = get_names(no_slow_results)
    slow_results = get_results_where(slow_results, lambda r: r["name"] in no_slow_names)
    no_slow_results.sort(key=by_name)
    slow_results.sort(key=by_name)
    print len(no_slow_results)
    print len(slow_results)
    slow_times = [result["stats"]["total_time_from_planner"] for result in slow_results]
    no_slow_times = [result["stats"]["total_time_from_planner"] for result in no_slow_results]
    slow_scores = [result["stats"]["score"] for result in slow_results]
    no_slow_scores = [result["stats"]["score"] for result in no_slow_results]
    print slow_times
    print no_slow_times
    time_differences = [a - b for a, b in zip(slow_times, no_slow_times)]
    print time_differences

    names = [result["name"] for result in slow_results]

    # plotPDF('../plots/slow_no_slow_score.pdf', slow_times, no_slow_times, names, "Effects of allowing slowing down on completion times")
    plotPDF('../plots/slow_no_slow_1/slow_no_slow_score.pdf', slow_scores, no_slow_scores, names, "Effects of allowing slowing down on scores")
