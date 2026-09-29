"""Strict configuration parameters shared by owner-specific schemas."""

from pydantic import BaseModel, ConfigDict


class Parameters(BaseModel):
    """Reject unknown fields and nonfinite numbers; prefer tuples for sequences."""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)
