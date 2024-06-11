#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pandas as pd


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


if __name__ == "__main__":
    data_frame = pd.read_csv("nvtl.csv")

    fig = plt.figure(figsize=(15, 5))
    axis = [fig.add_subplot(1, 4, i) for i in range(1, 5)]

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
    axis[0].plot(time_stamp, data_frame.iloc[:, 5], label="no dynamic points ")
    axis[0].plot(time_stamp, data_frame.iloc[:, 6], label="no ground & dynamic points")

    # plot position v.s. diff nvtl
    diff = data_frame.iloc[:, 5] - data_frame.iloc[:, 4]
    axis[1].plot(time_stamp, diff, label="no dynamic - raw")

    # plot position v.s. raw nvtl
    closure = lambda a, v, l: plot_scatter(a, nvtl_max, nvtl_min, position, v, l)
    closure(axis[2], data_frame.iloc[:, 4], "raw NVTL")
    scatter = closure(axis[3], data_frame.iloc[:, 5], "dynamic points excluded NVTL")
    fig.colorbar(scatter)

    # set legend
    [a.legend() for a in axis]
    [a.grid() for a in axis]

    axis[2].set_aspect("equal")
    axis[3].set_aspect("equal")

    plt.show()
