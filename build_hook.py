"""Hatch custom build hook: compile Zig shared library and set platform wheel tag."""

from __future__ import annotations

import platform
import shutil
import subprocess
import sys
from pathlib import Path

from hatchling.builders.hooks.plugin.interface import BuildHookInterface


class ZigBuildHook(BuildHookInterface):
    """Build hook that compiles the Zig native library and sets wheel tags."""

    PLUGIN_NAME = "zig-build"

    def initialize(self, version: str, build_data: dict) -> None:
        root = Path(self.root)
        lib_name = _lib_name()
        lib_src = root / "zig-out" / "lib" / lib_name
        lib_dst = root / "python" / "rt_lacam" / lib_name

        # Build Zig shared library if not already built or outdated
        if not lib_src.exists() or _sources_newer_than(root / "src", lib_src):
            _build_zig(root)

        if not lib_src.exists():
            print(
                f"ERROR: {lib_src} not found after zig build. "
                "Ensure Zig >= 0.15.0 is installed.",
                file=sys.stderr,
            )
            raise FileNotFoundError(str(lib_src))

        # Copy library into Python package
        shutil.copy2(lib_src, lib_dst)

        # Set platform-specific wheel tag (not pure Python)
        build_data["pure_python"] = False
        build_data["tag"] = _wheel_tag()


def _lib_name() -> str:
    system = platform.system()
    if system == "Darwin":
        return "librt_lacam.dylib"
    elif system == "Linux":
        return "librt_lacam.so"
    elif system == "Windows":
        return "rt_lacam.dll"
    return "librt_lacam.so"


def _wheel_tag() -> str:
    """Generate platform-specific wheel tag.

    Format: {python_tag}-{abi_tag}-{platform_tag}
    e.g. py3-none-macosx_14_0_arm64, py3-none-manylinux_2_17_x86_64
    """
    system = platform.system()
    machine = platform.machine()

    if system == "Darwin":
        # Use macOS 14.0 as minimum for arm64
        if machine == "arm64":
            return "py3-none-macosx_14_0_arm64"
        return "py3-none-macosx_13_0_x86_64"
    elif system == "Linux":
        if machine == "x86_64":
            return "py3-none-manylinux_2_17_x86_64"
        elif machine == "aarch64":
            return "py3-none-manylinux_2_17_aarch64"
    elif system == "Windows":
        if machine == "AMD64":
            return "py3-none-win_amd64"

    # Fallback
    plat = f"{system.lower()}_{machine}"
    return f"py3-none-{plat}"


def _build_zig(root: Path) -> None:
    zig = shutil.which("zig")
    if zig is None:
        raise RuntimeError(
            "Zig compiler not found. Install Zig >= 0.15.0: "
            "https://ziglang.org/download/"
        )

    print(f"Building rt-lacam native library with {zig}...")
    subprocess.run(
        [zig, "build", "-Doptimize=ReleaseFast"],
        cwd=root,
        check=True,
    )
    print("Zig build complete.")


def _sources_newer_than(src_dir: Path, target: Path) -> bool:
    if not target.exists():
        return True
    target_mtime = target.stat().st_mtime
    for f in src_dir.rglob("*.zig"):
        if f.stat().st_mtime > target_mtime:
            return True
    return False
