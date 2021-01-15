#!/usr/bin/env python3
"""Run the test suite for kimera_vio."""
import contextlib
import subprocess
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
        ["catkin", "locate", "-b"],
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
    package_path = get_package_dir()
    build_path = get_build_dir(package_path)
    test_location = build_path / "kimera_vio" / "testKimeraVIO"

    with tempfile.TemporaryDirectory(dir=str(package_path)) as run_dir:
        with change_directory(run_dir):
            subprocess.run([str(test_location)] + sys.argv[1:])


if __name__ == "__main__":
    main()
