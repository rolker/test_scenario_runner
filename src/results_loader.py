import os
import pickle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages


def load_config(path_to_test_dir):
    """Load all three parameter sets from a results directory. Returns in the order: planner, mpc, sim."""
    with open(path_to_test_dir + "/planner_config.pickle", "rb") as config_file:
        planner_config = pickle.load(config_file)
    with open(path_to_test_dir + "/mpc_config.pickle", "rb") as config_file:
        mpc_config = pickle.load(config_file)
    with open(path_to_test_dir + "/sim_config.pickle", "rb") as config_file:
        sim_config = pickle.load(config_file)
    return planner_config, mpc_config, sim_config


def load_stats(path_to_test_dir):
    with open(path_to_test_dir + "/stats.pickle", "rb") as stats_file:
        stats = pickle.load(stats_file)
    return stats


def load_all_results(path_to_results_dir, day_filters=None, day_not_filters=None):
    # beware, the filters use substrings, so if you give a day it could be a month too and vice versa
    days = next(os.walk(path_to_results_dir))
    # days = [os.path.join(d[0], d[1]) for d in days]

    day_names = days[1]
    # apply filters
    if day_filters:
        for f in day_filters:
            day_names = [d for d in day_names if f in d]
    if day_not_filters:
        for f in day_not_filters:
            day_names = [d for d in day_names if f not in d]

    test_folders = []
    test_names = []
    full_test_names = []
    for day in day_names:
        p = next(os.walk(os.path.join(days[0], day)))
        for test in p[1]:
            # TODO! -- apply test filters
            test_folders.append(os.path.join(p[0], test))
            test_names.append(test[9:-5])
            full_test_names.append(test)

    # tests = next(days)
    # test_names = tests[1]
    config = [load_config(p) for p in test_folders]
    stats = [load_stats(p) for p in test_folders]
    everything = [{
        "name": x[0],
        "path": x[3],
        "planner_config": x[1][0],
        "mpc_config": x[1][1],
        "sim_config": x[1][2],
        "stats": x[2],
    } for x in zip(test_names, config, stats, test_folders)]

    return everything


# access, filtering, etc
def by_name(result):
    return result["name"]


def remove_not_finished(results):
    return [result for result in results if result["stats"]["uncovered_length"] == 0]


def remove_names(results, names):
    return [result for result in results if result["name"] not in names]


def get_scores(results):
    return get_stats_item(results, "score")


def get_achievable_fractions(results):
    achievable_counts = np.array([sum(r) for r in get_stats_item(results, "last_plan_achievables")])
    iteration_counts = np.array([float(len(r)) for r in get_stats_item(results, "last_plan_achievables")])
    return achievable_counts / iteration_counts


def get_stats_item(results, stat_name):
    return [result["stats"][stat_name] for result in results]


def get_results_where(results, filter):
    return [result for result in results if filter(result)]


def get_results_with_specific_config_value(results, config_name, field, value):
    def f(result):
        return result[config_name][field] == value

    return get_results_where(results, f)


def run_aground(result):
    return result["stats"]["total_time_from_planner"] < result["stats"]["time_limit"] and\
           result["stats"]["uncovered_length"] > 0


def get_names(results):
    return [result["name"] for result in results]


# plotting
def plot_bar_chart(path, x, names, title, y_label, x_label):
    with PdfPages(path) as pdf:
        # change the size (not necessary) units are in inches
        # plt.figure(figsize=(10, 5))

        # change font size (make test readable)
        plt.rcParams["font.size"] = 8
        # force labels to be visible
        fig, ax = plt.subplots(tight_layout=True)

        # plot data...
        N = len(x)
        ind = np.arange(N)  # the x locations for the groups
        width = 0.27  # the width of the bars

        rects1 = ax.bar(ind, x, width, color='b')
        # rects2 = ax.bar(ind + width, x1, width, color='b', label=label1)

        ax.set_ylabel(y_label)
        # ax.set_yscale('log')
        ax.set_xticks(ind)
        ax.set_xticklabels(names)
        ax.set_xlabel(x_label)
        # rotate names so you can read them
        plt.xticks(rotation=90)

        # ax.legend(loc='upper right')
        # plt.show()

        plt.title(title)
        pdf.savefig()  # saves the current figure into a pdf page
        plt.close()


def plot_side_by_side_bar_chart(path, x1, x2, names, title, y_label, x_label, label1, label2, scale="log"):
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
        width = 0.27  # the width of the bars

        rects1 = ax.bar(ind, x2, width, color='r', label=label2)
        rects2 = ax.bar(ind + width, x1, width, color='b', label=label1)

        ax.set_ylabel(y_label)
        ax.set_yscale(scale)
        ax.set_xticks(ind + width / 2)
        ax.set_xticklabels(names)
        ax.set_xlabel(x_label)
        # rotate names so you can read them
        plt.xticks(rotation=90)

        ax.legend(loc='upper right')
        # plt.show()

        # plt.title(title)
        pdf.savefig()  # saves the current figure into a pdf page
        plt.close()


def plot_several_bar_charts_to_one_pdf(path, x_list, names, titles_list, y_label, x_label):
    with PdfPages(path) as pdf:
        # change the size (not necessary) units are in inches
        # plt.figure(figsize=(10, 5))

        for x, title in zip(x_list, titles_list):
            # change font size (make test readable)
            plt.rcParams["font.size"] = 8
            # force labels to be visible
            fig, ax = plt.subplots(tight_layout=True)

            # plot data...
            N = len(x)
            ind = np.arange(N)  # the x locations for the groups
            width = 0.27  # the width of the bars

            rects1 = ax.bar(ind, x, width, color='b')
            # rects2 = ax.bar(ind + width, x1, width, color='b', label=label1)

            ax.set_ylabel(y_label)
            # ax.set_yscale('log')
            ax.set_xticks(ind)
            ax.set_xticklabels(names)
            ax.set_xlabel(x_label)
            # rotate names so you can read them
            plt.xticks(rotation=90)

            # ax.legend(loc='upper right')
            # plt.show()

            plt.title(title)
            pdf.savefig()  # saves the current figure into a pdf page
            plt.close()


if __name__ == "__main__":
    # demo
    results = load_all_results("../results/")
    print len(results)
    for result in results:
        print result["name"], result["path"]
    print ("+++++++++++++++++++++++++++++++++++++")
    results = load_all_results("../results", day_filters=["2020_07_07"])
    print len(results)
    for result in results:
        print result["name"]

    print ("+++++++++++++++++++++++++++++++++++++")
    results = load_all_results("../results")
    cannon_results = [result for result in results if "cannon" in result["name"]]
    results = [result for result in results if "cannon" not in result["name"]]
    for result in results:
        print result["name"], result["stats"]["score"]
    for result in cannon_results:
        print result["name"], result["stats"]["score"]

    print ("+++++++++++++++++++++++++++++++++++++")
    results = load_all_results("../results")
    results = [result for result in results if result["planner_config"]["max_speed"] < 2]
    for result in results:
        print result["name"], result["stats"]["score"], result["planner_config"]["max_speed"]

    print ("+++++++++++++++++++++++++++++++++++++")
    # should ignore when we find best possible plan quickly...
    results = load_all_results("../results")
    lengths = [len(result["stats"]["iterations_counts"]) for result in results]
    times = [result["stats"]["score"] - result["stats"]["cumulative_collision_penalty"] for result in results]
    print zip(lengths, times)

