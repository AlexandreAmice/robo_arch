"""Validation of application-package resource references."""

from pathlib import Path


def validate_package_reference(reference: str, *, owner: Path | None = None) -> str:
    """Reject filesystem paths and ambiguous application-package resource URIs."""
    prefix = "package://robo_arch/"
    parts = reference.removeprefix(prefix).split("/")
    if (
        not reference.startswith(prefix)
        or any(part in {"", ".", ".."} for part in parts)
        or any(character in reference for character in "\\%?#")
    ):
        raise ValueError(
            f"Reference {reference!r}{f' in {owner}' if owner is not None else ''} must use "
            "package://robo_arch/<resource> without path traversal"
        )
    return reference
