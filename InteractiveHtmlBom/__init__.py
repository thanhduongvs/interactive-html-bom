"""Interactive HTML BOM.

KiCad integration uses the KiCad IPC API (see plugin.json in this
directory and ecad/kicad_ipc.py). The legacy SWIG ``pcbnew`` action plugin
has been removed because KiCad drops the ``pcbnew`` Python module in
version 11, so importing this package has no side effects.
"""
