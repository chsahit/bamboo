"""
Bamboo Franka Robot Client

A Python package for controlling Franka robots using joint impedance control
with ZMQ + msgpack communication to C++ control nodes.
"""

from importlib.metadata import metadata, version

from bamboo.client import BambooFrankaClient

__version__ = version("bamboo-franka-controller")
_metadata = metadata("bamboo-franka-controller")
_authors = _metadata.get_all("Author")
__author__ = ", ".join(_authors) if _authors else "Bamboo Development Team"

__all__ = ["BambooFrankaClient"]
