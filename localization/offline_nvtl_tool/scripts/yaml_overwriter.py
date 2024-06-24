#!/usr/bin/env python3
import shutil, os
import yaml
import argparse


class YamlOverwriter:
    def __init__(self, velocity_param_path, imu_param_path):
        self.imu_param_path = imu_param_path
        self.velocity_param_path = velocity_param_path

        if not os.path.exists(imu_param_path):
            raise FileNotFoundError("{0} does not exist".format(imu_param_path))

        if not os.path.exists(velocity_param_path):
            raise FileNotFoundError("{0} does not exist".format(velocity_param_path))

        copied_imu_param_path = imu_param_path + ".original"
        copied_velocity_param_path = velocity_param_path + ".original"
        self.copied_imu_param_path = copied_imu_param_path
        self.copied_velocity_param_path = copied_velocity_param_path

        if os.path.exists(copied_imu_param_path):
            print("skip copy because {0} already exists".format(copied_imu_param_path))
        else:
            shutil.copy(imu_param_path, copied_imu_param_path)

        if os.path.exists(copied_velocity_param_path):
            print("skip copy because {0} already exists".format(copied_velocity_param_path))
        else:
            shutil.copy(velocity_param_path, copied_velocity_param_path)

    def set_velocity_scale(self, velocity_scale_factor):
        with open(self.velocity_param_path, "r") as file:
            data = yaml.safe_load(file)

        data["/**"]["ros__parameters"]["speed_scale_factor"] = velocity_scale_factor

        with open(self.velocity_param_path, "w") as file:
            yaml.safe_dump(data, file)

    def set_imu_offset(self, imu_offset_z):
        with open(self.imu_param_path, "r") as file:
            data = yaml.safe_load(file)
        data["/**"]["ros__parameters"]["angular_velocity_offset_z"] = imu_offset_z

        with open(self.imu_param_path, "w") as file:
            yaml.safe_dump(data, file)

    def __del__(self):
        print("restore all parameter files")
        print(self.copied_velocity_param_path, self.velocity_param_path)
        shutil.move(self.copied_velocity_param_path, self.velocity_param_path)
        shutil.move(self.copied_imu_param_path, self.imu_param_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("vehicle_velocity_converter_param_path", help="path")
    parser.add_argument("imu_corrector_param_path", help="path")

    args = parser.parse_args()

    overwriter = YamlOverwriter(
        args.vehicle_velocity_converter_param_path, args.imu_corrector_param_path
    )
    overwriter.set_imu_offset(0.314)
    overwriter.set_velocity_scale(0.987)
    print("the parameter files are changed. press enter to restore the original files.")
    input()
