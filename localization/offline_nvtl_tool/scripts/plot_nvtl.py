#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pandas as pd
import argparse
import numpy as np


def plot_scatter(axis, nvtl_max, nvtl_min, position, value, label):
    return axis.scatter(
        position[:, 0],
        position[:, 1],
        vmax=nvtl_max,
        vmin=nvtl_min,
        c=value,
        cmap=cm.jet,
        label=label,
    )


def plot_comprehensive_nvtl(data_frame):
    fig = plt.figure(figsize=(15, 5))
    axis = fig.add_subplot(1, 2, 1)

    time_stamp = data_frame.iloc[:, 0] * 1e-9
    time_stamp = time_stamp - time_stamp[0]

    for index in range(13 * 13):
        column = int(index / 13)
        row = index % 13

        l1_distance = abs(6 - column) + abs(6 - row)

        color = cm.jet(l1_distance / 12)
        alpha = 0.2
        if l1_distance == 0:  # center
            alpha = 1.0

        axis.plot(time_stamp, data_frame.iloc[:, 7 + index], alpha=alpha, color=color)

    axis.set_title("comprehensive nvtl")
    axis.set_xlabel("time [s]")
    axis.set_ylabel("NVTL")
    axis.grid()

    # create data for boxplot
    data = [[] for i in range(13)]
    for index in range(13 * 13):
        column = int(index / 13)
        row = index % 13
        l1_distance = int(abs(6 - column) + abs(6 - row))

        c = data_frame.iloc[:, 7 + index].values.tolist()
        data[l1_distance].extend(c)

    axis2 = fig.add_subplot(1, 2, 2)
    axis2.boxplot(data, labels=[str(i) for i in range(13)])
    axis2.set_title("boxplot")
    axis2.set_xlabel("l1 distance of position offset")
    axis2.set_xlabel("NVTL")
    axis2.grid()


def plot_graph(input_path):
    data_frame = pd.read_csv(input_path)

    fig = plt.figure(figsize=(15, 5))
    axis = [
        fig.add_subplot(2, 2, 1),
        fig.add_subplot(2, 2, 3),
        fig.add_subplot(1, 4, 3),
        fig.add_subplot(1, 4, 4),
    ]

    # convert time to seconds
    time_stamp = data_frame.iloc[:, 0] * 1e-9
    time_stamp = time_stamp - time_stamp[0]

    # position
    position = data_frame.iloc[:, 1:3].values

    # max and min of nvtl
    nvtl_max = data_frame.iloc[:, 4:5].values.max()
    nvtl_min = data_frame.iloc[:, 4:5].values.min()

    # plot time v.s. nvtl
    axis[0].plot(time_stamp, data_frame.iloc[:, 4], label="raw")
    axis[0].plot(time_stamp, data_frame.iloc[:, 5], label="no_dynamic")
    axis[0].plot(time_stamp, data_frame.iloc[:, 6], label="no_dynamic_ground")

    # plot position v.s. diff nvtl
    diff = data_frame.iloc[:, 5] - data_frame.iloc[:, 4]
    axis[1].plot(time_stamp, diff, label="no_dynamic - raw")

    # plot position v.s. raw nvtl
    closure = lambda a, v, l: plot_scatter(a, nvtl_max, nvtl_min, position, v, l)
    closure(axis[2], data_frame.iloc[:, 4], "raw")
    scatter = closure(axis[3], data_frame.iloc[:, 5], "no_dynamic")
    fig.colorbar(scatter)

    # set legend
    [a.legend() for a in axis]
    [a.grid() for a in axis]

    axis[2].set_aspect("equal")
    axis[3].set_aspect("equal")

    plot_comprehensive_nvtl(data_frame)

    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="nvtl.csv")
    args = parser.parse_args()
    plot_graph(args.input)


if __name__ == "__main__":
    main()
