#!/usr/bin/env python3
"""Run the test suite for kimera_vio."""
import contextlib
import subprocess
import argparse
import tempfile
import pathlib
import sys
import os


@contextlib.contextmanager
def change_directory(new_path):
    """Change directory to desired path then revert."""
    current_directory = os.getcwd()
    os.chdir(str(new_path))
    try:
        yield
    finally:
        os.chdir(current_directory)


def get_package_dir() -> str:
    """Get the package directory for kimera_vio."""
    ret = subprocess.run(
        ["rospack", "find", "kimera_vio"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if ret.returncode != 0:
        raise RuntimeError(
            "catkin locate failed: {}".format(ret.stderr.decode("utf-8"))
        )

    return pathlib.Path(ret.stdout.decode("utf-8").strip("\n"))


def get_build_dir(package_path) -> str:
    """Get the build directory for the workspace."""
    ret = subprocess.run(
        ["catkin", "locate", "-b", "kimera_vio"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=str(package_path),
    )
    if ret.returncode != 0:
        raise RuntimeError(
            "catkin locate failed: {}".format(ret.stderr.decode("utf-8"))
        )

    return pathlib.Path(ret.stdout.decode("utf-8").strip("\n"))


def main():
    """Locate test binary and switch to that."""
    parser = argparse.ArgumentParser(
        description="""Utility to run gtest for kimera_vio.
Additional args (i.e. anything besides --prefix and --use_gdb) are passed to gtest."""
    )
    parser.add_argument(
        "--prefix",
        "-p",
        nargs="?",
        type=str,
        default=None,
        help="prefix for running test executable with additional profiling e.g. gdb",
    )
    parser.add_argument(
        "--use_gdb", "-g", action="store_true", help="run unit tests with gdb"
    )
    args, unknown = parser.parse_known_args()

    package_path = get_package_dir()
    ws_path = package_path.parent.parent
    build_path = get_build_dir(ws_path)
    test_location = build_path / "testKimeraVIO"

    prefix = []
    if args.prefix is not None:
        prefix = args.prefix.split(" ")
    elif args.use_gdb:
        prefix = ["gdb", "--args"]

    with tempfile.TemporaryDirectory(dir=str(package_path)) as run_dir:
        with change_directory(run_dir):
            subprocess.run(prefix + [str(test_location)] + unknown)


if __name__ == "__main__":
    main()
