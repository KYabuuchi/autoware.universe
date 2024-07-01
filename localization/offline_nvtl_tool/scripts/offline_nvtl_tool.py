#!/usr/bin/env python3
import argparse
from validate import validate_trajectory
import subprocess
import make_trajectory


def evaluate_rosbag(args: argparse.Namespace, recored_rosbag_path: str):

    launch_offline_score_command = 'ros2 launch offline_nvtl_tool offline_nvtl_tool.launch.xml \
        rosbag_path:="{0}" \
        pcd_path:={1} \
        output_csv_path:={2}'.format(
        recored_rosbag_path, args.map_path, args.score_path
    )
    print(launch_offline_score_command)

    # launch in background
    return subprocess.run(launch_offline_score_command, shell=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", help="path to rosbag")
    parser.add_argument("map_path", help="path to pcd map")
    parser.add_argument("sensor_model", help="e.g. sample_sensor_kit")
    parser.add_argument("vehicle_model", help="e.g. sample_vehicle")
    parser.add_argument("vehicle_id", help="")

    parser.add_argument("vehicle_velocity_converter_param_path", help="path")
    parser.add_argument("imu_corrector_param_path", help="path")
    parser.add_argument("--score_path", "-s", default="/tmp/nvtl.csv")
    args = parser.parse_args()

    for velocity_scale in [0.95, 1.0, 1.05]:
        # (1) make_trajectory.py
        recorded_rosbag_path = make_trajectory.main(
            args.rosbag_path,
            args.map_path,
            args.sensor_model,
            args.vehicle_model,
            args.vehicle_id,
            args.vehicle_velocity_converter_param_path,
            args.imu_corrector_param_path,
            velocity_scale=velocity_scale,
        )

        # (2) evaluate rosbag and produce score csv
        evaluate_rosbag(args, recorded_rosbag_path)

        # (3) validate score csv
        is_successed = validate_trajectory(args.score_path)
        if is_successed:
            print(
                "SUCCESS:{0} with scale_factor={1}",
                recorded_rosbag_path,
                velocity_scale,
            )
        else:
            print("FAILED")


if __name__ == "__main__":
    main()
