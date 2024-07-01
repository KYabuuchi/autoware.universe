#!/usr/bin/env python3
import pandas as pd
import argparse
import numpy as np
import matplotlib.pyplot as plt

# TODO: Very temporary value. This should be adjusted in the future.
SCORE_DIFF_TORELANCE = -0.1


def validate_trajectory(nvtl_csv_path: str, enable_plot: bool = False) -> bool:
    data_frame = pd.read_csv(nvtl_csv_path)

    all_score_data = []
    target_score = None

    for index in range(13 * 13):
        column = int(index / 13)
        row = index % 13

        l1_distance = abs(6 - column) + abs(6 - row)

        if l1_distance != 0:
            all_score_data.append(data_frame.iloc[:, 7 + index].values.tolist())
        else:
            target_score = data_frame.iloc[:, 7 + index].values.tolist()

    all_score_data = np.array(all_score_data)
    target_score = np.array(target_score)

    envelope_around_score = np.max(all_score_data, axis=0)

    success_timing = (target_score - envelope_around_score) > SCORE_DIFF_TORELANCE

    if enable_plot:
        fig = plt.figure(figsize=(15, 5))
        axis = fig.add_subplot(1, 1, 1)
        axis.plot(target_score, label="target")
        axis.plot(envelope_around_score, label="envelope")
        axis.plot(success_timing, label="success=1, fail=0")
        axis.legend()
        plt.show()

    # If always target score  is highr than around score, return True. Otherwise, return False.
    return np.all(success_timing)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("score_csv_path", help="nvtl.csv")
    args = parser.parse_args()

    is_successed = validate_trajectory(args.score_csv_path, True)
    if is_successed:
        print("Success")
    else:
        print("Failed")


if __name__ == "__main__":
    main()
