#!/usr/bin/env python3
import argparse
from validate import validate_trajectory
import subprocess


def evaluate_rosbag(args: argparse.Namespace):

    launch_offline_score_command = 'ros2 launch offline_nvtl_tool offline_nvtl_tool.launch.xml \
        rosbag_path:="{0}" \
        pcd_path:={1} \
        output_csv_path:={2}'.format(
        args.rosbag_path, args.map_path, args.score_path
    )
    print(launch_offline_score_command)

    # launch in background
    return subprocess.run(launch_offline_score_command, shell=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--score_path", "-s", default="/tmp/nvtl.csv")
    parser.add_argument("rosbag_path", help="path to rosbag")
    parser.add_argument("map_path", help="path to pcd map")
    args = parser.parse_args()

    # TODO: add here calling make_trajectory.py

    # (1) evaluate rosbag and produce score csv
    evaluate_rosbag(args)

    # (2) validate score csv
    is_successed = validate_trajectory(args.score_path)
    if is_successed:
        print("Success")
    else:
        print("Failed")


if __name__ == "__main__":
    main()
