#!/usr/bin/env python3
# -------------------------------------------------------------------------- #
# * Copyright 2019, Massachusetts Institute of Technology,
# * Cambridge, MA 02139
# * All Rights Reserved
# * Authors: Luca Carlone, et al. (see THANKS for the full author list)
# * See LICENSE for the license information
# -------------------------------------------------------------------------- #
"""
Creates proper configurations for real hardware from a Kalibr calibration.

This script creates yaml config files for Kimera-VIO from a Kalibr camera and
IMU calibration.

Authors:
    * Sandro Berchier
    * Antoni Rosinol
    * Nathan Hughes

Example:
    You can run this module as a script like so::
        python kalibr_params_to_kimera_params.py -i /path/to/kalibr/camera_imu/output \
                                                 -o /path/to/new/config/home \
                                                 -a Responsible Person

    Run ``python kalibr_params_to_kimera_params.py --help`` for more information

"""

import numpy as np
import argparse
import datetime
import logging
import pathlib
import jinja2
import yaml
import sys


VALID_MODELS = ["stereo-radtan", "stereo-equi"]

CAMERA_MAPPINGS = {
    "T_BS": ("T_cam_imu", lambda x: np.linalg.inv(np.array(x))),
    "camera_width": ("resolution", lambda x: x[0]),
    "camera_height": ("resolution", lambda x: x[1]),
    "camera_model": ("camera_model", None),
    "u_focal_length": ("intrinsics", lambda x: x[0]),
    "v_focal_length": ("intrinsics", lambda x: x[1]),
    "u_principal_point": ("intrinsics", lambda x: x[2]),
    "v_principal_point": ("intrinsics", lambda x: x[3]),
    "distortion_model": ("distortion_model", None),
    "distortion_coefficients": ("distortion_coeffs", np.array),
}

IMU_MAPPINGS = {
    "T_BS": ("T_i_b", lambda x: np.linalg.inv(np.array(x))),
    "rate_hz": ("update_rate", None),
    "gyroscope_noise_density": ("gyroscope_noise_density", None),
    "gyroscope_random_walk": ("gyroscope_random_walk", None),
    "accelerometer_noise_density": ("accelerometer_noise_density", None),
    "accelerometer_random_walk": ("accelerometer_random_walk", None),
}


def remap_kalibr_config(kalibr_config, mapping):
    """Map relevant fields from kalibr to kimera."""
    kimera_config = {}

    for kimera_key, kalibr_key_map_pair in mapping.items():
        kalibr_key, transform_func = kalibr_key_map_pair
        if kalibr_key in kalibr_config:
            if transform_func is None:
                kimera_config[kimera_key] = kalibr_config[kalibr_key]
            else:
                kimera_config[kimera_key] = transform_func(kalibr_config[kalibr_key])
        else:
            logging.warning("Could not find key: " + kalibr_key + ". Ignoring entry...")

    return kimera_config


def make_header(values, width=80, comment_character="#"):
    """Create a pretty-print header as a jinja filter."""
    bookend = comment_character * width
    lines = ["# {}: {}".format(key, value) for key, value in values.items()]
    lines = [line + " " * (width - len(line) - 1) + "#" for line in lines]
    lines = [bookend] + lines + [bookend]
    return "\n".join(lines)


def numpy_display(value, flat=True, format_str=" 1.8f"):
    """Create a properly-formatted array as a jinja filter."""
    if type(value) is not np.ndarray:
        return str(value)

    # punt on arrays with more than 2 dimensions
    if len(value.shape) > 2:
        return str(value)

    # punt on empty arrays
    if value.size == 0:
        return str(value)

    # make a lambda function for converting floats to a string
    float_format = ("{{:{}}}".format(format_str)).format

    with np.printoptions(floatmode="fixed", suppress=True):
        # avoid messy logic for vectors by forcing flat representation
        is_flat = len(value.shape) == 1
        can_be_flat = is_flat or value.shape[0] == 1 or value.shape[1] == 1

        if flat or can_be_flat:
            # return single line vector
            elements = np.squeeze(value).tolist()
            contents = ", ".join(map(float_format, elements))
            return "[{}]".format(contents[1:] if contents[0] == " " else contents)

        # flatten each line and assemble a list
        to_render = [
            ", ".join(map(float_format, np.squeeze(row).tolist())) for row in value
        ]
        contents = ",\n".join(to_render)

        # handle weirdness at start of list
        if contents[0] == " ":
            return " [{}]".format(contents[1:])
        else:
            return "[{}]".format(contents)


def get_jinja_env():
    """Make a jinja environment that loads files from the templates directory."""
    template_path = pathlib.Path(__file__).absolute().parent / "templates"
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(str(template_path)))
    env.filters["numpy_display"] = numpy_display
    env.filters["make_header"] = make_header

    return env


def load_kalibr_information_from_files(camera_file, imu_file):
    """Load information from files."""
    with open(camera_file, "r") as fin:
        kalibr_camera = yaml.load(fin.read(), Loader=yaml.SafeLoader)

    with open(imu_file, "r") as fin:
        kalibr_imu = yaml.load(fin.read(), Loader=yaml.SafeLoader)

    return kalibr_camera, kalibr_imu


def load_kalibr_information(result_directory):
    """Detect and load kalibr results from a directory."""
    result_path = pathlib.Path(result_directory)
    potential_camchains = list(result_path.glob("camchain-*imu*cam*.yaml"))
    potential_imus = list(result_path.glob("imu*.yaml"))

    if len(potential_camchains) == 0:
        raise RuntimeError("could not find camchain file")

    if len(potential_camchains) > 1:
        raise RuntimeError("too many camchain files present")

    if len(potential_imus) == 0:
        raise RuntimeError("could not find imu file")

    if len(potential_imus) > 1:
        raise RuntimeError("too many imu files present")

    return load_kalibr_information_from_files(
        str(potential_camchains[0]), str(potential_imus[0])
    )


def make_camera_config(kalibr_camera, kalibr_camera_id, camera_id, extra_information):
    """Make a camera config."""
    kimera_config = remap_kalibr_config(
        kalibr_camera[kalibr_camera_id], CAMERA_MAPPINGS
    )
    kimera_config["camera_id"] = camera_id
    kimera_config.update(extra_information)
    return kimera_config


def make_imu_config(kalibr_imu, kalibr_imu_id, extra_information):
    """Make an imu config."""
    kimera_config = remap_kalibr_config(kalibr_imu[kalibr_imu_id], IMU_MAPPINGS)

    if "time_offset" in kalibr_imu[kalibr_imu_id]:
        kimera_config["imu_time_shift"] = kalibr_imu[kalibr_imu_id]["time_offset"]
    else:
        kimera_config["imu_time_shift"] = 0.0

    kimera_config.update(extra_information)
    return kimera_config


def get_args():
    """Construct an argparser and return parsed arguments."""
    parser = argparse.ArgumentParser(description="tool to convert calibration formats")

    parser.add_argument(
        "-o", "--output", help="output path (default cwd)", default=None
    )
    parser.add_argument(
        "-a", "--responsible", help="person creating configuration", default=None
    )

    parser.add_argument(
        "-i", "--input_directory", help="path to kalibr result directory", default=None
    )
    parser.add_argument(
        "--input_imu", help="explicit path to imu configuration", default=None
    )
    parser.add_argument(
        "--input_camera", help="explicit path to camera configuration", default=None
    )

    parser.add_argument("-r", "--camera_rate", help="camera rate (in hz)", default=20)
    parser.add_argument("-n", "--camera_name", help="name of camera", default=None)
    parser.add_argument(
        "--imu_name", help="name of imu (if different from camera)", default=None
    )

    parser.add_argument(
        "--kalibr_camera_ids",
        help="kalibr camera ids",
        nargs="+",
        default=["cam0", "cam1"],
    )
    parser.add_argument("--kalibr_imu_id", help="kalibr imu id", default="imu0")

    parser.add_argument(
        "--imu_bias_init_sigma", help="initial bias covariance", default=1.0e-3
    )
    parser.add_argument(
        "--imu_integration_sigma", help="integration covariance", default=1.0e-8
    )
    parser.add_argument(
        "-g", "--gravity", help="gravity vector", nargs=3, default=[0.0, 0.0, -9.81]
    )

    return parser.parse_args()


def verify_args(args):
    """Make sure we have a valid set of arguments."""
    input_dir_valid = args.input_directory is not None
    input_files_valid = args.input_imu is not None and args.input_camera is not None
    all_invalid = not input_dir_valid and not input_files_valid

    if all_invalid:
        logging.critical("must either provide an input directory or two input files.")
        sys.exit(1)

    if input_dir_valid and input_files_valid:
        logging.warning("both a directory and files specified. Will default to files")

    return input_files_valid


def output_kimera_configs(
    output, imu_config, left_camera_config, right_camera_config=None
):
    """Write configurations to disk."""
    env = get_jinja_env()
    imu_template = env.get_template("imu.yaml")
    camera_template = env.get_template("camera.yaml")

    if output is not None:
        output_path = pathlib.Path(output)
    else:
        output_path = pathlib.Path(".").absolute()

    with (output_path / "ImuParams.yaml").open("w") as fout:
        fout.write(imu_template.render(**imu_config))

    with (output_path / "LeftCameraParams.yaml").open("w") as fout:
        fout.write(camera_template.render(**left_camera_config))

    if right_camera_config is None:
        return

    with (output_path / "RightCameraParams.yaml").open("w") as fout:
        fout.write(camera_template.render(**right_camera_config))


def main():
    """Parse arguments and run everything."""
    logging.basicConfig(
        format="%(levelname)s [%(asctime)s]: %(message)s",
        datefmt="%m/%d/%Y %I:%M:%S %p",
    )

    args = get_args()
    input_files_valid = verify_args(args)

    if input_files_valid:
        kalibr_camera, kalibr_imu = load_kalibr_information_from_files(
            args.input_camera, args.input_imu
        )
    else:
        kalibr_camera, kalibr_imu = load_kalibr_information(args.input_directory)

    metadata = {
        "Created by": args.responsible if args.responsible is not None else "Unkown",
        "Created on": datetime.datetime.now().isoformat(),
    }
    if args.camera_name is not None:
        metadata["Camera"] = args.camera_name
    if args.imu_name is not None:
        metadata["IMU"] = args.imu_name

    extra_information = {"metadata": metadata}
    extra_cam_information = extra_information.copy()

    extra_cam_information["rate_hz"] = args.camera_rate

    extra_imu_information = extra_information.copy()
    extra_imu_information["imu_integration_sigma"] = args.imu_integration_sigma
    extra_imu_information["imu_bias_init_sigma"] = args.imu_bias_init_sigma
    extra_imu_information["n_gravity"] = np.array(args.gravity)

    kimera_imu = make_imu_config(kalibr_imu, args.kalibr_imu_id, extra_imu_information)

    kimera_left_camera = make_camera_config(
        kalibr_camera, args.kalibr_camera_ids[0], "left_cam", extra_cam_information
    )

    kimera_right_camera = None
    if len(args.kalibr_camera_ids) > 1:
        kimera_right_camera = make_camera_config(
            kalibr_camera, args.kalibr_camera_ids[1], "right_cam", extra_cam_information
        )

    output_kimera_configs(
        args.output, kimera_imu, kimera_left_camera, kimera_right_camera
    )


if __name__ == "__main__":
    main()
