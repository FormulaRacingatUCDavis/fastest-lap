#!/usr/bin/env python3
"""Collect the recursive MinGW DLL dependencies of a Windows runtime."""

import argparse
import os
import pathlib
import re
import shutil
import subprocess


DLL_PATTERN = re.compile(r"^\s*DLL Name:\s*(\S+)\s*$", re.IGNORECASE)


def dll_imports(objdump, binary):
    result = subprocess.run(
        [str(objdump), "-p", str(binary)],
        check=True,
        capture_output=True,
        text=True,
        errors="replace",
    )
    return [
        match.group(1)
        for line in result.stdout.splitlines()
        if (match := DLL_PATTERN.match(line))
    ]


def directory_index(directories):
    index = {}
    for directory in directories:
        if not directory.is_dir():
            continue
        for candidate in directory.iterdir():
            if candidate.is_file() and candidate.suffix.lower() == ".dll":
                index.setdefault(candidate.name.lower(), candidate.resolve())
    return index


def is_windows_system_dll(name):
    lowered = name.lower()
    if lowered.startswith(("api-ms-win-", "ext-ms-win-")):
        return True

    windows = pathlib.Path(os.environ.get("WINDIR", r"C:\Windows"))
    return any(
        (directory / name).is_file()
        for directory in (windows / "System32", windows / "SysWOW64")
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--objdump", required=True, type=pathlib.Path)
    parser.add_argument("--destination", required=True, type=pathlib.Path)
    parser.add_argument("--binary", required=True, action="append", type=pathlib.Path)
    parser.add_argument("--search-dir", action="append", default=[], type=pathlib.Path)
    args = parser.parse_args()

    objdump = args.objdump.resolve()
    if not objdump.is_file():
        raise FileNotFoundError(objdump)

    destination = args.destination.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    # Prefer freshly built/search-path DLLs over an older copy that may already
    # exist in the staging destination after an incremental relink.
    search_directories = [path.resolve() for path in args.search_dir] + [destination]
    available = directory_index(search_directories)

    queue = []
    for binary in args.binary:
        resolved = binary.resolve()
        if not resolved.is_file():
            raise FileNotFoundError(resolved)
        queue.append(resolved)

    visited = set()
    copied = []
    unresolved = []

    while queue:
        binary = queue.pop(0)
        binary_key = str(binary).lower()
        if binary_key in visited:
            continue
        visited.add(binary_key)

        for imported_name in dll_imports(objdump, binary):
            imported_key = imported_name.lower()
            source = available.get(imported_key)
            if source is None:
                if is_windows_system_dll(imported_name):
                    continue
                unresolved.append((binary, imported_name))
                continue

            target = (destination / source.name).resolve()
            if source != target:
                shutil.copy2(source, target)
                copied.append(target)
                available[imported_key] = target
            queue.append(target)

    if unresolved:
        details = "\n".join(
            f"  {binary.name}: {dependency}" for binary, dependency in unresolved
        )
        raise RuntimeError(f"Unresolved non-system DLL dependencies:\n{details}")

    msys_runtime = destination / "msys-2.0.dll"
    if msys_runtime.exists():
        raise RuntimeError(
            "The runtime depends on msys-2.0.dll; build it with MINGW64, not MSYS."
        )

    print(f"Audited {len(visited)} PE binaries; copied {len(copied)} runtime DLLs.")
    for path in sorted(set(copied)):
        print(path)


if __name__ == "__main__":
    main()
