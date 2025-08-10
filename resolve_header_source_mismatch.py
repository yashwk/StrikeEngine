import os

SRC_DIR = 'src/strikeengine'
INCLUDE_DIR = 'include/strikeengine'


def collect_files(root, ext):
    result = set()
    for base, _, files in os.walk(root):
        for file in files:
            if file.endswith(ext):
                rel_path = os.path.relpath(os.path.join(base, file), root)
                result.add(rel_path.replace('\\', '/'))
    return result


def ensure_directory(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)


def create_header(path):
    ensure_directory(path)
    with open(path, 'w') as f:
        f.write("#pragma once\n\n")
        f.write(f"// Auto-generated header for {os.path.basename(path)}\n")


def create_source(path, header_rel_path):
    ensure_directory(path)
    with open(path, 'w') as f:
        f.write(f'#include "{header_rel_path}"\n\n')
        f.write(f"// Auto-generated source for {os.path.basename(path)}\n")


cpp_files = collect_files(SRC_DIR, '.cpp')
hpp_files = collect_files(INCLUDE_DIR, '.hpp')

cpp_stems = {f[:-4] for f in cpp_files}
hpp_stems = {f[:-4] for f in hpp_files}

missing_headers = cpp_stems - hpp_stems
missing_sources = hpp_stems - cpp_stems

print("\n=== Auto Resolving Header/Source Mismatches ===\n")

for name in sorted(missing_headers):
    header_path = os.path.join(INCLUDE_DIR, name + ".hpp").replace("/", os.sep)
    print(f"🛠  Creating missing header: {header_path}")
    create_header(header_path)

for name in sorted(missing_sources):
    source_path = os.path.join(SRC_DIR, name + ".cpp").replace("/", os.sep)
    header_rel_path = f"strikeengine/{name}.hpp"
    print(f"🛠  Creating missing source: {source_path}")
    create_source(source_path, header_rel_path)

print("\n Resolution complete.\n")
