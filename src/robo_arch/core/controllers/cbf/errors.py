"""Explicit filter failure shared by execution adapters."""


class CbfFailure(RuntimeError):
    """Stop the calling run, preserving a numerical failure snapshot."""

    def __init__(self, reason: str, **snapshot):
        self.snapshot = snapshot
        super().__init__(f"CBF {reason}: {snapshot}")
