#!/usr/bin/env python3
"""Build script: compile Zig library, copy into package, build wheel.

Usage:
    python build_wheel.py          # build wheel
    python build_wheel.py --sdist  # build sdist only
"""

from __future__ import annotations

import platform
import shutil
import subprocess
import sys
from pathlib import Path


def lib_name() -> str:
    system = platform.system()
    if system == "Darwin":
        return "librt_lacam.dylib"
    elif system == "Linux":
        return "librt_lacam.so"
    elif system == "Windows":
        return "rt_lacam.dll"
    return "librt_lacam.so"


def main() -> None:
    root = Path(__file__).parent
    name = lib_name()
    lib_src = root / "zig-out" / "lib" / name
    lib_dst = root / "python" / "rt_lacam" / name

    # Step 1: Build Zig library (ReleaseFast)
    zig = shutil.which("zig")
    if zig is None:
        print("ERROR: Zig compiler not found. Install from https://ziglang.org/download/")
        sys.exit(1)

    print(f"Building with {zig}...")
    subprocess.run([zig, "build", "-Doptimize=ReleaseFast"], cwd=root, check=True)

    if not lib_src.exists():
        print(f"ERROR: {lib_src} not found after build")
        sys.exit(1)

    # Step 2: Copy library into Python package
    shutil.copy2(lib_src, lib_dst)
    print(f"Copied {name} -> {lib_dst}")

    # Step 3: Build wheel
    if "--sdist" in sys.argv:
        subprocess.run([sys.executable, "-m", "build", "--sdist"], cwd=root, check=True)
    else:
        subprocess.run([sys.executable, "-m", "build", "--wheel"], cwd=root, check=True)


if __name__ == "__main__":
    main()
