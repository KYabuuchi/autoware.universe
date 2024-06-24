#!/usr/bin/env python3
import argparse
import subprocess
import time
import threading
from datetime import datetime
import yaml_overwriter


def change_odometry_parameter(args: argparse.Namespace, scale=0.95):
    # TODO: use yaml_overwriter.py
    pass


def launch_autoware(args: argparse.Namespace):
    launch_autoware_command = "ros2 launch autoware_launch logging_simulator.launch.xml \
        map_path:={0} \
        vehicle_model:={1}\
        vehicle_id:={2}\
        sensor_model:={3}\
        vehicle:=true \
        system:=false \
        map:=true \
        sensing:=true \
        localization:=true \
        perception:=true\
        planning:=false \
        control:=false".format(
        args.map, args.vehicle_model, args.vehicle_id, args.sensor_model
    )

    # launch in background
    return subprocess.Popen(launch_autoware_command, shell=True)


def wait_for_autoware_ready():
    service_call_command = 'ros2 service call /localization/pose_estimator/trigger_node std_srvs/srv/SetBool \
        "{data: false}"'

    # run in forground
    subprocess.run(service_call_command, shell=True)

    time.sleep(3)


def record_rosbag():
    # get current time like 2021_01_01-12_00_00
    formatted_date_time = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    recorded_rosbag_path = "/tmp/rosbag2_{0}".format(formatted_date_time)

    record_rosbag_command = "ros2 bag record -o {0} --use-sim-time\
        /map/pointcloud_map \
        /localization/pose_estimator/pose_with_covariance \
        /localization/util/downsample/pointcloud \
        /localization/kinematic_state \
        /tf \
        /tf_static \
        /perception/object_recognition/detection/centerpoint/objects \
        /perception/object_recognition/detection/objects".format(
        recorded_rosbag_path
    )

    # start recording in background
    return subprocess.Popen(record_rosbag_command, shell=True)


def play_rosbag(args: argparse.Namespace):
    print("play rosbag")

    play_rosbag_command = 'ros2 bag play "{0}" \
         -r 0.10 --clock 200 --topics \
         /sensing/gnss/ublox/nav_sat_fix \
         /sensing/imu/tamagawa/imu_raw \
         /sensing/lidar/top/velodyne_packets \
         /vehicle/status/velocity_status'.format(
        args.rosbag
    )
    print(play_rosbag_command)

    subprocess.run(play_rosbag_command, shell=True)

    # wait to finish


def embed_rosbag_speed_reset():
    topic_name = "/localization/initialization_state"

    echo_command = "ros2 topic echo {0} --once".format(topic_name)
    reset_speed_command = (
        'ros2 service call /rosbag2_player/set_rate rosbag2_interfaces/SetRate "rate: 1.0"'
    )

    while True:
        time.sleep(1)
        echo_result = subprocess.run(
            echo_command, capture_output=True, text=True, shell=True
        ).stdout

        if type(echo_result) != str:
            print("echo_reulst is not str")
            continue

        print(echo_result)
        if "state: 3" in echo_result:
            print("reset rosbag play speed")
            subprocess.run(reset_speed_command, shell=True)
            break


def kill_autoware():
    subprocess.run("pkill ros", shell=True)
    subprocess.run("pkill rviz", shell=True)
    subprocess.run("pkill aggregator_node", shell=True)
    subprocess.run(
        'ps aux | grep "ros-args" | grep -v grep | awk \'{ print "kill ", $2 }\' | sh', shell=True
    )
    subprocess.run(
        "ps aux | grep component_container | grep -v grep | awk '{ print \"kill \", $2 }' | sh",
        shell=True,
    )
    subprocess.run(
        "ps aux | grep robot_state_publisher| grep -v grep | awk '{ print \"kill \", $2 }' | sh",
        shell=True,
    )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("rosbag", help="path to rosbag")
    parser.add_argument("map", help="path to map")
    parser.add_argument("sensor_model", help="e.g. sample_sensor_kit")
    parser.add_argument("vehicle_model", help="e.g. sample_vehicle")
    parser.add_argument("vehicle_id", help="")
    parser.add_argument("--vehicle_velocity_converter_param_path", help="path")
    parser.add_argument("--imu_corrector_param_path", help="path")
    args = parser.parse_args()

    process_list = []
    try:
        # Assumes that the user has already sourced the Autoware workspace
        # 1. Change imu/vehicle velocity parameter
        # change_odometry_parameter(args)
        # 2. Launch Autoware
        process_list.append(launch_autoware(args))
        # 3. Wait for Autoware to be ready
        wait_for_autoware_ready()
        # 4. Record the localization result to rosbag
        process_list.append(record_rosbag())
        # 5. Start another thread to reset rosbag playback speed when the localization initialization is complete
        thread = threading.Thread(target=embed_rosbag_speed_reset)
        thread.start()
        # 6. Play rosbag at slow speed (Blocking call)
        play_rosbag(args)
        # 7. When rosbag is finished, stop recording and kill autoware
        print("Success: finish all process")
    except:
        print("Error: finish all process")
    finally:
        kill_autoware()


if __name__ == "__main__":
    main()
