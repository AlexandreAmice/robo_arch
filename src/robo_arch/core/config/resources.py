"""Validation of application-package resource references."""

from pathlib import Path


def validate_package_reference(reference: str, *, owner: Path | None = None) -> str:
    """Validate resource syntax without opening files or importing an SDK.

    :param reference: Required ``package://robo_arch/<resource>`` URI.
    :param owner: Optional referring file, used only in error messages.
    :returns: The unchanged reference; existence is not checked.
    :raises ValueError: Empty/dot path segments, traversal, backslashes, percent
        escapes, queries, fragments or a different URI prefix.
    """
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
