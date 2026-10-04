"""SDK-independent declarations for rigid protection spheres and barriers."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Sphere:
    """Rigid protection sphere in a named frame.

    :param name: Nonempty sphere identifier used by pair declarations.
    :param frame: ``world`` for a fixed center, or ``instance/frame`` (including
        nested instance names) for a controller-model frame.
    :param center: Three finite coordinates in that frame, in metres.
    :param radius: Finite, strictly positive radius in metres.
    :raises ValueError: Empty names, invalid center or radius.

    Frame existence and uniqueness within an assembly are checked by the runtime.
    """

    name: str
    frame: str
    center: tuple[float, float, float]
    radius: float

    def __post_init__(self) -> None:
        if not self.name or not self.frame:
            raise ValueError("Sphere name and frame must be nonempty")
        if len(self.center) != 3 or not all(map(math.isfinite, self.center)):
            raise ValueError("Sphere center must contain three finite meters")
        if not math.isfinite(self.radius) or self.radius <= 0:
            raise ValueError("Sphere radius must be finite and positive")


@dataclass(frozen=True)
class SpherePair:
    """One enabled pair of named spheres.

    :param first: Name of the first sphere.
    :param second: Distinct name of the second sphere.
    :param margin: Finite nonnegative clearance beyond both radii, in metres.
    :raises ValueError: Empty/equal names or invalid margin.

    This record does not resolve sphere names or add any unlisted pairs.
    """

    first: str
    second: str
    margin: float = 0.01

    def __post_init__(self) -> None:
        if not self.first or not self.second or self.first == self.second:
            raise ValueError("A sphere pair requires two distinct names")
        if not math.isfinite(self.margin) or self.margin < 0:
            raise ValueError("Pair margin must be finite and nonnegative")


@dataclass(frozen=True)
class Plane:
    """Fixed world halfspace ``normal @ position >= offset``; offset in meters.

    Normal points into the permitted halfspace and must already have unit length.

    :param name: Nonempty plane identifier, distinct from sphere names at assembly.
    :param normal: Three finite world components, unit length within 1e-10.
    :param offset: Finite signed plane offset in metres.
    :raises ValueError: Invalid name, normal or offset; normals are not normalized.
    """

    name: str
    normal: tuple[float, float, float]
    offset: float

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("Plane name must be nonempty")
        if len(self.normal) != 3 or not all(map(math.isfinite, self.normal)):
            raise ValueError("Plane normal must contain three finite components")
        if not math.isclose(math.hypot(*self.normal), 1.0, rel_tol=0, abs_tol=1e-10):
            raise ValueError("Plane normal must have unit length")
        if not math.isfinite(self.offset):
            raise ValueError("Plane offset must be finite")


@dataclass(frozen=True)
class SpherePlanePair:
    """Select a sphere and fixed plane for halfspace protection.

    :param sphere: Nonempty sphere name.
    :param plane: Nonempty plane name, distinct from the sphere name.
    :param margin: Finite nonnegative extra surface clearance, in metres.
    :raises ValueError: Empty/equal names or invalid margin.

    Defining a plane alone adds no constraint; a pair is required.
    """

    sphere: str
    plane: str
    margin: float = 0.01

    def __post_init__(self) -> None:
        if not self.sphere or not self.plane or self.sphere == self.plane:
            raise ValueError("A sphere-plane pair requires two distinct names")
        if not math.isfinite(self.margin) or self.margin < 0:
            raise ValueError("Pair margin must be finite and nonnegative")


@dataclass(frozen=True)
class CbfParameters:
    """Linear barrier gains in s^-1 and numerical acceptance tolerance.

    Tolerance applies separately to effort bounds (N m) and the CBF residual
    (m²/s² for sphere pairs; m/s² for planes). Initial h and psi1 use 1e-10
    in each row's respective units: m², m²/s or m, m/s.

    :param alpha1: Positive finite gain in ``psi1 = hdot + alpha1*h``.
    :param alpha2: Positive finite gain in ``psi1dot + alpha2*psi1 >= 0``.
    :param residual_tolerance: Positive finite runtime acceptance tolerance.
        The numerical row builders do not apply it or check initial feasibility.
    :raises ValueError: Any nonpositive or nonfinite setting.
    """

    alpha1: float = 5.0
    alpha2: float = 5.0
    residual_tolerance: float = 1e-6

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(value) and value > 0
            for value in (self.alpha1, self.alpha2, self.residual_tolerance)
        ):
            raise ValueError("CBF gains and residual tolerance must be positive")
