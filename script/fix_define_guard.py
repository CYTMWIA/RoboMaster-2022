#! /usr/bin/python3

import os

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

def make_guard(path):
    parts = path.strip().split('/')
    for i in range(len(parts)-1, -1,-1):
        if parts[i] in ["include", "src"]:
            parts = parts[i+1:]
            break
    guard = "_".join(parts)
    for c in [".", "\\", "/"]:
        guard = guard.replace(c, "_")
    guard = guard.upper()
    while not guard.startswith("__"):
        guard = "_"+guard
    while not guard.endswith("__"):
        guard = guard+"_"

    return guard

def main():
    project_dir = os.path.dirname(os.path.dirname((os.path.abspath(__file__))))
    os.chdir(project_dir)
    
    files = ls_all_file("src")
    headers = list(filter(lambda p:p.endswith(".hpp"), files))
    
    for header in headers:
        with open(header, "r", encoding="utf-8") as f:
            lines = f.readlines()

        guard = make_guard(header)

        break_count = 2
        for l in range(len(lines)):
            line = lines[l].strip()
            for word in ["#define", "#ifndef"]:
                if line.startswith(word):
                    lines[l] = f"{word} {guard}\n"
                    break_count -= 1
            if break_count == 0:
                break
        
        with open(header, "w", encoding="utf-8") as f:
            f.writelines(lines)

if __name__=="__main__":
    main()