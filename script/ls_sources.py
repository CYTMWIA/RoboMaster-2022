#! /usr/bin/python3

import os

import common

def main():
    files = common.ls_sources()

    for f in files:
        print(f)

if __name__=="__main__":
    main()