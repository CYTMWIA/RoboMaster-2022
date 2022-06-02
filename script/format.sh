#! /usr/bin/bash

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
cd ${SCRIPTPATH}/../

if ! command -v clang-format &> /dev/null
then
    echo "clang-format could not be found"
    exit
fi

python3 ./script/ls_sources.py | xargs clang-format -style=file -i
