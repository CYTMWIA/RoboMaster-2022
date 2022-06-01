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

def filename_to_lowercase(filename):
    res = ""
    idx = 0
    while idx<len(filename):
        letter = filename[idx]
        if letter.lower()==letter:
            res += letter
            idx += 1
        else:
            count = 1
            while filename[idx+count].lower()!=filename[idx+count]:
                count += 1
            if idx==0:
                res += filename[idx:idx+count].lower()
            else:
                res += "_" + filename[idx:idx+count].lower()
            idx += count
    return res

def main():
    project_dir = os.path.dirname(os.path.dirname((os.path.abspath(__file__))))
    os.chdir(project_dir)
    
    files = ls_all_file("src")
    files = list(filter(lambda p: any([p.endswith(ext) for ext in [".h", ".hpp", ".c", ".cpp"]]), files))
    files_need_rename = list(filter(lambda f: any([l.lower()!=l for l in f.split("/")[-1]]), files))

    filename2lower = {
        os.path.basename(file): filename_to_lowercase(os.path.basename(file)) for file in files_need_rename
    }

    for file in files:
        with open(file, "r", encoding="utf-8") as f:
            lines = f.readlines()
        
        for idx, line in enumerate(lines):
            line = line.strip()
            if line.startswith("#include"):
                include = line[len("#include"):].strip()
                if include[0] == include[-1] == "\"":
                    include = include[1:-1]
                    inc_dir = os.path.dirname(include)
                    inc_filename = os.path.basename(include)
                    if inc_filename in filename2lower:
                        include_new = os.path.join(inc_dir, filename2lower[inc_filename])
                        include_new = "#include \"" + include_new + "\"\n"
                        lines[idx] = include_new
                else:
                    continue
        
        with open(file, "w", encoding="utf-8") as f:
            f.writelines(lines)
    
    for file in files_need_rename:
        new_path = os.path.join(os.path.dirname(file), filename2lower[os.path.basename(file)])
        os.rename(file, new_path)

    # print(filename2lower)
    



if __name__=="__main__":
    main()