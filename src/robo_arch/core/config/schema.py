"""Common validation policy for SDK-independent configuration models."""

from pydantic import BaseModel, ConfigDict


class Schema(BaseModel):
    """Reject unknown fields and nonfinite numbers; prevent field reassignment.

    Freezing is shallow: nested dictionaries remain mutable. Prefer tuples for
    sequences and treat loaded parameter dictionaries as read-only.
    """

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)
