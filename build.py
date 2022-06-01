#! /usr/bin/python3

import argparse
import os
import sys
import time


def build(rebuild):
    if rebuild:
        os.system("rm -rf CMake* src")

    return os.system("cmake -DARMOR_MODEL=openvino_yolox .. && make -j$(($(nproc) + 1))")


def run(debug, auto_restart):
    os.system("ln -s ../asset/* ./")

    cmd = "./REDO"
    if debug:
        cmd += " --debug"

    os.system(cmd)
    while auto_restart:
        os.system(
            "./usbreset $(lsusb | grep Dahua | awk 'NR==1{gsub(\"[^0-9]\", \" \", $0); print \"/dev/bus/usb/\"$1\"/\"$2}')")
        os.system(
            "./usbreset $(lsusb | grep Dheng | awk 'NR==1{gsub(\"[^0-9]\", \" \", $0); print \"/dev/bus/usb/\"$1\"/\"$2}')")
        time.sleep(5)
        os.system(cmd)


def main():
    parser = argparse.ArgumentParser(description="构建脚本")
    parser.add_argument("--rebuild", action='store_true', default=False)
    parser.add_argument("--run", action='store_true', default=False)
    parser.add_argument("--auto-restart", action='store_true', default=False)
    parser.add_argument("--debug", action='store_true', default=False)
    arg = parser.parse_args()

    this_path = sys.argv[0]
    work_dir = os.path.join(os.path.dirname(this_path), "build")
    os.makedirs(work_dir, exist_ok=True)
    os.chdir(work_dir)

    # build
    last_return = build(arg.rebuild)
    # run
    if last_return == 0 and arg.run:
        last_return = run(arg.debug, arg.auto_restart)


if __name__ == "__main__":
    main()
