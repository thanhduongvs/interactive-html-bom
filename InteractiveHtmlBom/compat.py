"""Compatibility utilities for optional wx dependency."""

import os

_wx = None
_wx_import_error = None


def should_create_wx_app():
    """Check if we should create a wx app based on environment."""
    return 'INTERACTIVE_HTML_BOM_NO_DISPLAY' not in os.environ


def get_wx():
    """Get the wx module, or None if not available."""
    global _wx
    if _wx is not None:
        return _wx
    try:
        import wx
        _wx = wx
        return _wx
    except ImportError as e:
        global _wx_import_error
        _wx_import_error = e
        return None


def get_wx_import_error():
    """The ImportError raised by the last failed get_wx(), or None."""
    return _wx_import_error
