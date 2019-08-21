#!/usr/bin/env python

import os
import sys
import argparse
import argcomplete
import yaml
import csv
import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import rc
#rc('font', **{'family': 'sans-serif', 'sans-serif': ['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
#rc('text', usetex=True)


def _set_boxplot_colors(boxplot_object, color):
    setp(boxplot_object['boxes'][0], color=color)
    setp(boxplot_object['caps'][0], color=color)
    setp(boxplot_object['caps'][1], color=color)
    setp(boxplot_object['whiskers'][0], color=color)
    setp(boxplot_object['whiskers'][1], color=color)
    #setp(boxplot_object['fliers'], color=color)
    setp(boxplot_object['medians'][0], color=color)


def draw_boxplot(axis, bxpstats):
    """
        bxpstats : list of dicts
          A list of dictionaries containing stats for each boxplot.
          Required keys are:
              - ``med``: The median (scalar float).

              - ``q1``: The first quartile (25th percentile) (scalar
                float).

              - ``q3``: The third quartile (75th percentile) (scalar
                float).

              - ``whislo``: Lower bound of the lower whisker (scalar
                float).

              - ``whishi``: Upper bound of the upper whisker (scalar
                float).

          Optional keys are:
              - ``mean``: The mean (scalar float). Needed if
                ``showmeans=True``.

              - ``fliers``: Data beyond the whiskers (sequence of floats).
                Needed if ``showfliers=True``.

              - ``cilo`` & ``cihi``: Lower and upper confidence intervals
                about the median. Needed if ``shownotches=True``.

              - ``label``: Name of the dataset (string). If available,
                this will be used a tick label for the boxplot

            positions : array-like, default = [1, 2, ..., n]
          Sets the positions of the boxes. The ticks and limits
          are automatically set to match the positions.
    """
    pb = axis.bxp(bxpstats,
                  widths=0.8,
                  vert=True,
                  showcaps=True,
                  showbox=True,
                  showfliers=False)


def plot_statistics_vio(statistics, output_boxplot_path):
    len_statistics = len(statistics.items())
    names = []
    maxes = []
    mins = []
    means = []
    samples = []
    stddevs = []
    stats = dict()
    for key, value in statistics.items():
        print('Reading statistic: %s' % str(key))
        if key == 'Pipeline Overall Timing [ms]':
            name = str(key[:-11])
            names.append(name)
            stats[name] = dict()
            for key, value in value.items():
                if key == 'max':
                    stats[name]['max'] = value
                    maxes.append(value)
                if key == 'min':
                    stats[name]['min'] = value
                    mins.append(value)
                if key == 'median':
                    stats[name]['median'] = value
                if key == 'q1':
                    stats[name]['q1'] = value
                if key == 'q3':
                    stats[name]['q3'] = value
                if key == 'samples':
                    stats[name]['samples'] = value
                    samples.append(value)
                if key == 'mean':
                    stats[name]['mean'] = value
                    means.append(value)
                if key == 'stddev':
                    stats[name]['stddev'] = value
                    stddevs.append(value)
        elif 'Timing [ms]' in str(key):
            name = str(key[:-11])
            names = [name] + names
            stats[name] = dict()
            for key, value in value.items():
                if key == 'max':
                    stats[name]['max'] = value
                    maxes = [value] + maxes
                if key == 'min':
                    stats[name]['min'] = value
                if key == 'median':
                    stats[name]['median'] = value
                    mins = [value] + mins
                if key == 'q1':
                    stats[name]['q1'] = value
                if key == 'q3':
                    stats[name]['q3'] = value
                if key == 'samples':
                    stats[name]['samples'] = value
                    samples = [value] + samples
                if key == 'mean':
                    stats[name]['mean'] = value
                    means = [value] + means
                if key == 'stddev':
                    stats[name]['stddev'] = value
                    stddevs = [value] + stddevs
        else:
            print(
                "Skipping this statistic, as it is not a Timing or is not in [ms]."
            )

    # Create stacked errorbars:
    #plt.errorbar(np.arange(len_statistics), np.asarray(means), np.asarray(stddevs), fmt='ok', lw=3)
    #plt.errorbar(np.arange(len(names)), np.asarray(means), [np.asarray(means) - np.asarray(mins), np.asarray(maxes) - np.asarray(means)],
    #             fmt='xk', ecolor='blue', lw=2, capsize=5, capthick=3, mfc='red', mec='green', ms=10, mew=4)

    fig, axes = plt.subplots()
    bxpstats = []
    for name, value in stats.items():
        bxpstats_a = dict()
        bxpstats_a['mean'] = value['mean']
        bxpstats_a['med'] = value['median']
        bxpstats_a['q1'] = value['q1']
        bxpstats_a['q3'] = value['q3']
        bxpstats_a['whislo'] = value['min']
        bxpstats_a['whishi'] = value['max']
        bxpstats.append(bxpstats_a)
    draw_boxplot(axes, bxpstats)

    # Formatting
    bar_width = 0.35
    opacity = 0.8

    locs, labels = plt.xticks()
    plt.xticks(range(len_statistics), names)  # Set locations and labels
    plt.title("Mean VIO timing per module (& max/min).")
    plt.ylabel('Time [ms]')
    plt.xlabel('VIO Module', labelpad=10)
    plt.show()


def parser():
    basic_desc = "Plot timing statistics for VIO pipeline."
    main_parser = argparse.ArgumentParser(description="{}".format(basic_desc))
    input_options = main_parser.add_argument_group("input options")
    input_options.add_argument(
        "statistics_vio_path",
        help="Path to the **YAML** file containing the VIO statistics.",
        default="./results/V1_01_easy/S/StatisticsVIO.yaml")
    input_options.add_argument(
        "output_boxplot_path",
        help="Path where to save boxplot file containing the VIO statistics.",
        default="./results/V1_01_easy/S/StatisitcsVIOboxplots.eps")
    return main_parser


def main(statistics_vio_path, output_boxplot_path):
    # Read vio statistics yaml file.
    print("Reading VIO statistics from: %s" % statistics_vio_path)
    if os.path.exists(statistics_vio_path):
        with open(statistics_vio_path, 'r') as input:
            statistics = yaml.load(input)
            plot_statistics_vio(statistics, output_boxplot_path)


if __name__ == "__main__":
    parser = parser()
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    main(args.statistics_vio_path, args.output_boxplot_path)
    sys.exit(os.EX_OK)
