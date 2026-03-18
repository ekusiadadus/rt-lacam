"""rt-lacam: Real-Time LaCAM for Multi-Agent Path Finding.

Zig-powered incremental MAPF solver with Python bindings via cffi.
"""

from rt_lacam._bindings import RTLaCAM

__all__ = ["RTLaCAM"]
__version__ = "0.2.0"
