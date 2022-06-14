#! /usr/bin/python3

import os

def project_dir():
    return os.path.dirname(os.path.dirname((os.path.abspath(__file__))))

def ls(path):
    return [os.path.join(path, p) for p in os.listdir(path)]

def ls_all_file(path):
    paths = ls(path)
    res = []
    for p in paths:
        if os.path.isdir(p):
            res += ls_all_file(p)
        else:
            res.append(p)
    return res

def ls_sources():
    files = ls_all_file(os.path.join(project_dir(), "src"))
    files = list(filter(
        lambda p:
            not any([ignore in p for ignore in ["3rdparty/", "build/", "install/"]]),
        files
    ))
    files = list(filter(
        lambda p:
            any([p.endswith(ext) for ext in [".h", ".hpp", ".c", ".cpp"]]),
        files
    ))
    return files
