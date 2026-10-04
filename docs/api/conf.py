"""Build the reference from installed APIs, with warnings treated as errors."""

from sphinx.application import Sphinx

project = "robo_arch"
extensions = ["sphinx.ext.autodoc"]
autodoc_typehints = "none"
html_theme = "alabaster"
exclude_patterns = []


def check_public_entry(
    app: Sphinx,
    what: str,
    name: str,
    obj: object,
    options: dict,
    lines: list[str],
) -> None:
    """Give every selected public object one canonical reference entry."""
    if not name.startswith("robo_arch.") or any(
        part.startswith("_") for part in name.split(".")
    ):
        raise ValueError(f"Not a public robo_arch API: {name}")
    if not lines:
        raise ValueError(f"Missing public documentation: {name}")
    seen = app._documented_objects
    if id(obj) in seen:
        raise ValueError(f"Duplicate public API entry or alias: {name}")
    seen.add(id(obj))


def setup(app: Sphinx) -> None:
    app._documented_objects = set()
    app.connect("autodoc-process-docstring", check_public_entry)
