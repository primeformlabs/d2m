"""APT standard tool definitions with ray-tool intersection formulas.

Supports the APT 7-parameter tool model with analytic ray-intersection
for each tool type.  All tools are defined in their local coordinate frame:
tip at origin, tool axis along +Z.

Reference: ANSI/ISO 4343, APT Programmer's Reference Manual
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np

from .tri_dexel import DexelInterval, DexelColumn, AABB


# =========================================================================
#   Helper: quadratic solver
# =========================================================================

def _solve_quadratic(a: float, b: float, c: float) -> Optional[Tuple[float, float]]:
    """Solve a * t^2 + 2 * b * t + c = 0 for t.

    Returns (t0, t1) with t0 <= t1, or None if no real roots.
    """
    if abs(a) < 1e-20:
        # Degenerate: linear equation  2*b*t + c = 0
        if abs(b) < 1e-20:
            return None
        t = -c / (2.0 * b)
        return (t, t)

    disc = b * b - a * c
    if disc < 0.0:
        return None

    sqrt_disc = np.sqrt(disc)
    t0 = (-b - sqrt_disc) / a
    t1 = (-b + sqrt_disc) / a

    if t0 > t1:
        t0, t1 = t1, t0
    return (t0, t1)


# =========================================================================
#   Ray-primitive intersection helpers
# -------------------------------------------------------------------------
#   All functions:
#     - take ray_origin O (3,), ray_direction D (3,), |D| = 1
#     - return a list of (t_entry, t_exit) interval tuples
#     - D is NOT required to be unit-length but should be for correct t
# =========================================================================

def _ray_cylinder(O: np.ndarray, D: np.ndarray,
                  radius: float, z_min: float, z_max: float) -> list[Tuple[float, float]]:
    """Intersection of a ray with a right circular cylinder.

    Cylinder: x^2 + y^2 <= radius^2,  z in [z_min, z_max].

    Returns list of (t_start, t_end) intervals inside the cylinder.
    """
    Ox, Oy, Oz = O[0], O[1], O[2]
    Dx, Dy, Dz = D[0], D[1], D[2]

    a = Dx * Dx + Dy * Dy
    b = Ox * Dx + Oy * Dy
    c = Ox * Ox + Oy * Oy - radius * radius

    def _clip_positive(t_in: float, t_out: float) -> Optional[Tuple[float, float]]:
        """Clip an interval to t >= 0. Returns None if entirely behind the ray."""
        if t_out < -1e-12:
            return None
        if t_in < -1e-12:
            t_in = 0.0
        if t_out < t_in:
            return None
        return (t_in, t_out)

    intervals: list[Tuple[float, float]] = []

    if abs(a) < 1e-20:
        # Ray is parallel to cylinder axis (Dx = Dy = 0)
        if c > 1e-12:
            return []  # outside the cylinder
        # Inside the infinite cylinder; find z-plane crossings
        if abs(Dz) < 1e-20:
            # Ray is perpendicular to axis AND inside cylinder:
            # infinite intersection (degenerate) -- treat as full z-range from 0
            return [(0.0, np.inf)]
        t_zmin = (z_min - Oz) / Dz
        t_zmax = (z_max - Oz) / Dz
        if t_zmin > t_zmax:
            t_zmin, t_zmax = t_zmax, t_zmin
        clipped = _clip_positive(t_zmin, t_zmax)
        if clipped:
            intervals.append(clipped)
        return intervals

    roots = _solve_quadratic(a, b, c)
    if roots is None:
        return []

    t0, t1 = roots
    # Compute z at each root and xy position
    z0 = Oz + t0 * Dz
    z1 = Oz + t1 * Dz

    # Check which segments are within [z_min, z_max]
    in_0 = (z_min <= z0 <= z_max)
    in_1 = (z_min <= z1 <= z_max)

    if in_0 and in_1:
        clipped = _clip_positive(t0, t1)
        if clipped:
            intervals.append(clipped)
    elif in_0 and not in_1:
        if z1 < z_min:
            t_bound = (z_min - Oz) / Dz if abs(Dz) > 1e-20 else t1
        else:
            t_bound = (z_max - Oz) / Dz if abs(Dz) > 1e-20 else t1
        if t0 > t_bound:
            t0, t_bound = t_bound, t0
        clipped = _clip_positive(t0, t_bound)
        if clipped:
            intervals.append(clipped)
    elif not in_0 and in_1:
        if z0 < z_min:
            t_bound = (z_min - Oz) / Dz if abs(Dz) > 1e-20 else t0
        else:
            t_bound = (z_max - Oz) / Dz if abs(Dz) > 1e-20 else t0
        if t_bound > t1:
            t_bound, t1 = t1, t_bound
        clipped = _clip_positive(t_bound, t1)
        if clipped:
            intervals.append(clipped)
    else:
        # Both outside -- check if ray passes through z-slab
        if z0 < z_min and z1 > z_max:
            t_in = (z_min - Oz) / Dz if abs(Dz) > 1e-20 else t0
            t_out = (z_max - Oz) / Dz if abs(Dz) > 1e-20 else t1
        elif z0 > z_max and z1 < z_min:
            t_in = (z_min - Oz) / Dz if abs(Dz) > 1e-20 else t1
            t_out = (z_max - Oz) / Dz if abs(Dz) > 1e-20 else t0
        else:
            return []
        if t_in > t_out:
            t_in, t_out = t_out, t_in
        clipped = _clip_positive(t_in, t_out)
        if clipped:
            intervals.append(clipped)

    return intervals


def _ray_sphere(O: np.ndarray, D: np.ndarray,
                center: np.ndarray, radius: float,
                clip_z_max: Optional[float] = None) -> list[Tuple[float, float]]:
    """Intersection of a ray with a sphere.

    Sphere: centre C, radius R.
    If clip_z_max is given, only keep intersection points where z <= clip_z_max
    (used to restrict the sphere to the lower hemisphere for ball endmills).

    Returns list of (t_start, t_end) intervals.
    """
    w = O - center
    a = 1.0  # |D| = 1
    b = np.dot(D, w)
    c = np.dot(w, w) - radius * radius

    roots = _solve_quadratic(a, b, c)
    if roots is None:
        return []

    t0, t1 = roots

    # Clip negative (behind-ray) solutions
    if t1 < -1e-12:
        return []
    if t0 < -1e-12:
        t0 = 0.0

    if clip_z_max is not None:
        z0 = O[2] + t0 * D[2]
        z1 = O[2] + t1 * D[2]
        in_0 = z0 <= clip_z_max + 1e-12
        in_1 = z1 <= clip_z_max + 1e-12

        if not in_0 and not in_1:
            return []
        if in_0 and not in_1:
            # Clip t1 at the z=clip_z_max plane
            if abs(D[2]) > 1e-20:
                t_clip = (clip_z_max - O[2]) / D[2]
                if t0 <= t_clip <= t1:
                    t1 = t_clip
                else:
                    return []
            else:
                return [(t0, t1)]  # no z-component, can't clip
        if not in_0 and in_1:
            if abs(D[2]) > 1e-20:
                t_clip = (clip_z_max - O[2]) / D[2]
                if t0 <= t_clip <= t1:
                    t0 = t_clip
                else:
                    return []
            else:
                return [(t0, t1)]

    return [(t0, t1)]


def _ray_cone(O: np.ndarray, D: np.ndarray,
              apex: np.ndarray, half_angle_rad: float,
              height: float, truncate_radius_min: float = 0.0) -> list[Tuple[float, float]]:
    """Intersection of a ray with a right circular cone.

    Cone: apex at `apex`, axis +Z, half-angle `half_angle_rad`.
    The cone extends from z=apex_z to z=apex_z+height.
    Radius at height h = h * tan(half_angle).

    If truncate_radius_min > 0, the cone starts with a minimum radius
    (truncated cone / frustum).

    Returns list of (t_start, t_end) intervals.
    """
    # Transform ray so apex is at origin
    Ox, Oy, Oz = O[0] - apex[0], O[1] - apex[1], O[2] - apex[2]
    Dx, Dy, Dz = D[0], D[1], D[2]

    k2 = np.tan(half_angle_rad) ** 2

    a = Dx * Dx + Dy * Dy - k2 * Dz * Dz
    b = Ox * Dx + Oy * Dy - k2 * Oz * Dz
    c = Ox * Ox + Oy * Oy - k2 * Oz * Oz

    roots = _solve_quadratic(a, b, c)
    if roots is None:
        return []

    t0, t1 = roots

    intervals: list[Tuple[float, float]] = []
    # Check which roots are within the height bounds
    for t in (t0, t1):
        z_t = Oz + t * Dz
        r_t = np.sqrt(k2) * z_t  # radius at this z
        # z must be in [0, height]
        if 0.0 <= z_t <= height + 1e-9:
            # radius must be >= truncate min
            if r_t >= truncate_radius_min - 1e-9:
                intervals.append((t, t))

    if len(intervals) == 2:
        t0, t1 = intervals[0][0], intervals[1][0]
        if t0 > t1:
            t0, t1 = t1, t0
        # Clip negative
        if t1 < -1e-12:
            return []
        if t0 < -1e-12:
            t0 = 0.0
        return [(t0, t1)]
    elif len(intervals) == 1:
        t_val = intervals[0][0]
        if t_val < -1e-12:
            return []
        return [(0.0, t_val)]  # ray origin inside cone
    else:
        return []


def _ray_disc(O: np.ndarray, D: np.ndarray,
              z_plane: float, radius: float) -> list[float]:
    """Intersection of a ray with a flat disc at z=z_plane, radius R.

    Returns list of t values where the ray hits the disc.
    Since the disc has zero thickness, each hit is a single t value.

    For a ray coplanar with the disc (Dz=0, Oz=z_plane), returns
    t values for intersection with the bounding circle.
    """
    if abs(D[2]) > 1e-20:
        t = (z_plane - O[2]) / D[2]
        if t < -1e-12:
            return []  # behind the ray
        px = O[0] + t * D[0]
        py = O[1] + t * D[1]
        if px * px + py * py <= radius * radius + 1e-12:
            return [t]
        return []
    else:
        # Ray is parallel to the disc plane
        if abs(O[2] - z_plane) > 1e-9:
            return []  # Ray misses the plane entirely
        # Ray is in the disc plane -- find intersections with circle
        a = D[0]**2 + D[1]**2
        b = O[0]*D[0] + O[1]*D[1]
        c = O[0]**2 + O[1]**2 - radius**2
        roots = _solve_quadratic(a, b, c)
        if roots is None:
            return []
        return list(roots)


# =========================================================================
#   APT 7-parameter tool definition
# =========================================================================

@dataclass
class ToolParameters:
    """APT 7-parameter tool geometry definition.

    Attributes:
        diameter:      Cutting diameter D (mm).
        corner_radius: Corner radius R (mm).  0 for sharp corner.
        length:        Total tool length L (mm).
        flute_length:  Length of cutting flutes FL (mm).
        taper_angle:   Side taper angle A (degrees).  0 = straight.
        tip_angle:     Bottom angle B (degrees).  0 = flat, 90 = full radius.
        tip_diameter:  Diameter at tip d (mm).  For drills, chamfers.
    """
    diameter: float
    corner_radius: float = 0.0
    length: float = 100.0
    flute_length: float = 50.0
    taper_angle: float = 0.0
    tip_angle: float = 0.0
    tip_diameter: float = 0.0


# =========================================================================
#   Abstract Tool base class
# =========================================================================

class Tool(ABC):
    """Abstract base class for all cutting tools.

    The tool coordinate system:
    - Tip at origin (0, 0, 0)
    - Tool axis along +Z (the tool points upward; cutting moves downward)
    - Cutting portion extends from z=0 to z=flute_length
    """

    def __init__(self, params: ToolParameters):
        self.params = params

    @property
    def diameter(self) -> float:
        return self.params.diameter

    @property
    def radius(self) -> float:
        return self.diameter / 2.0

    @property
    def length(self) -> float:
        return self.params.length

    @property
    def flute_length(self) -> float:
        return self.params.flute_length

    @property
    def corner_radius(self) -> float:
        return self.params.corner_radius

    # -----------------------------------------------------------------
    #  Ray intersection (abstract)
    # -----------------------------------------------------------------

    @abstractmethod
    def ray_intersection(self, ray_origin: np.ndarray,
                         ray_direction: np.ndarray) -> DexelColumn:
        """Compute the intersection interval(s) of this tool with a ray.

        The ray is given in **tool-local** coordinates (tip at origin, +Z up).

        Returns a DexelColumn with sorted entry/exit intervals.
        May be empty if the ray misses the tool entirely.
        """
        ...

    # -----------------------------------------------------------------
    #  Swept-volume bbox (abstract)
    # -----------------------------------------------------------------

    @abstractmethod
    def swept_volume_bbox(self, start_pose, end_pose) -> AABB:
        """Bounding box of the volume swept between two tool poses."""
        ...

    # -----------------------------------------------------------------
    #  Local bbox
    # -----------------------------------------------------------------

    def bbox_at_origin(self) -> AABB:
        """Axis-aligned bounding box of the tool in its local frame."""
        r = self.radius
        return AABB(min=np.array([-r, -r, 0.0]),
                     max=np.array([r, r, self.flute_length]))

    # -----------------------------------------------------------------
    #  Shared helper: collect t values into a DexelColumn
    # -----------------------------------------------------------------

    @staticmethod
    def _collect_intervals(
        interval_pairs: list[Tuple[float, float]],
    ) -> DexelColumn:
        """Merge a list of (t_start, t_end) pairs into a DexelColumn.

        Filters zero-length and negative intervals.
        """
        valid: list[DexelInterval] = []
        for s, e in interval_pairs:
            if e - s > 1e-12:
                valid.append(DexelInterval(s, e))
        if not valid:
            return DexelColumn()
        # Sort by start, merge overlaps
        return DexelColumn.from_intervals_list(
            [(iv.start, iv.end) for iv in valid]
        )

    def __repr__(self) -> str:
        return (f"{type(self).__name__}(D={self.diameter:.1f}, "
                f"R={self.corner_radius:.1f}, FL={self.flute_length:.1f})")


# =========================================================================
#   Flat Endmill
# =========================================================================

class FlatEndmill(Tool):
    """Flat endmill: cylinder + flat bottom disc.

    APT:  D > 0,  R = 0,  A = 0,  B = 0.

    Geometry:
    - Cylinder  radius= D/2   from z=0  to z=FL
    - Bottom disc  radius= D/2   at z=0
    """

    def ray_intersection(self, ray_origin: np.ndarray,
                         ray_direction: np.ndarray) -> DexelColumn:
        """Ray-flat-endmill intersection.

        Algorithm:
        1. Intersect ray with infinite cylinder of radius D/2.
           Clip the intersection interval(s) to z in [0, FL].
        2. Intersect ray with the bottom disc at z=0, radius D/2.
        3. Collect all valid t-values; the global min/max define the
           single entry/exit interval (the tool is convex).
        """
        O = np.asarray(ray_origin, dtype=np.float64)
        D = np.asarray(ray_direction, dtype=np.float64)
        # Normalise direction
        d_norm = np.linalg.norm(D)
        if d_norm < 1e-20:
            return DexelColumn()
        D = D / d_norm

        R = self.radius
        FL = self.flute_length

        all_t: list[float] = []

        # 1. Cylinder
        cyl_intervals = _ray_cylinder(O, D, R, 0.0, FL)
        for s, e in cyl_intervals:
            all_t.extend([s, e])

        # 2. Bottom disc
        disc_t = _ray_disc(O, D, 0.0, R)
        all_t.extend(disc_t)

        if len(all_t) < 2:
            return DexelColumn()

        all_t.sort()
        # For a convex solid the first and last t are entry/exit
        return DexelColumn([DexelInterval(all_t[0], all_t[-1])])

    def swept_volume_bbox(self, start_pose, end_pose) -> AABB:
        # Stub -- full implementation in Phase 3
        return self.bbox_at_origin()


# =========================================================================
#   Ball Endmill  (most common 5-axis tool)
# =========================================================================

class BallEndmill(Tool):
    """Ball endmill: cylinder + hemisphere at tip.

    APT:  D > 0,  R = D/2,  A = 0,  B = 90.

    Geometry:
    - Cylinder    radius= D/2   from z=R      to z=FL
    - Hemisphere  radius= D/2   centre at (0, 0, R),  z <= R
    """

    def __init__(self, diameter: float, flute_length: float = 50.0,
                 length: float = 100.0):
        params = ToolParameters(
            diameter=diameter,
            corner_radius=diameter / 2.0,
            length=length,
            flute_length=flute_length,
            tip_angle=90.0,
        )
        super().__init__(params)

    def ray_intersection(self, ray_origin: np.ndarray,
                         ray_direction: np.ndarray) -> DexelColumn:
        """Ray-ball-endmill intersection.

        Two components whose union almost always gives one continuous
        interval:
        1. Cylinder  radius=R  from z=R  to z=FL.
        2. Sphere    radius=R  centre (0,0,R),  capped at z <= R (hemisphere).

        The two shapes are tangent at the equator (z=R) so the union
        is always a single interval per ray.
        """
        O = np.asarray(ray_origin, dtype=np.float64)
        D = np.asarray(ray_direction, dtype=np.float64)
        d_norm = np.linalg.norm(D)
        if d_norm < 1e-20:
            return DexelColumn()
        D = D / d_norm

        R = self.radius
        FL = self.flute_length

        all_t: list[float] = []

        # 1. Cylinder  [z=R .. z=FL]
        cyl_intervals = _ray_cylinder(O, D, R, R, FL)
        for s, e in cyl_intervals:
            all_t.extend([s, e])

        # 2. Sphere (hemisphere: z <= R)
        sphere_centre = np.array([0.0, 0.0, R])
        sph_intervals = _ray_sphere(O, D, sphere_centre, R, clip_z_max=R)
        for s, e in sph_intervals:
            all_t.extend([s, e])

        if len(all_t) < 2:
            return DexelColumn()

        all_t.sort()
        return DexelColumn([DexelInterval(all_t[0], all_t[-1])])

    def swept_volume_bbox(self, start_pose, end_pose) -> AABB:
        # Stub -- full implementation in Phase 3
        return self.bbox_at_origin()

    def bbox_at_origin(self) -> AABB:
        R = self.radius
        return AABB(min=np.array([-R, -R, -R]),
                     max=np.array([R, R, self.flute_length]))


# =========================================================================
#   Bull Nose Endmill
# =========================================================================

class BullNoseEndmill(Tool):
    """Bull nose endmill: cylinder + toroidal corner + flat bottom.

    APT:  D > 0,  0 < R_corner < D/2,  A = 0,  B = 0.

    Geometry:
    - Cylinder    radius = D/2             z in [R_c, FL]
    - Flat bottom radius = D/2 - R_corner   z = 0
    - Torus       major radius = R_corner,  minor radius = D/2 - R_corner
                  centred at (0, 0, R_corner), sweeping from z=0 to z=R_corner
    """

    def ray_intersection(self, ray_origin: np.ndarray,
                         ray_direction: np.ndarray) -> DexelColumn:
        """Ray-bull-nose intersection.

        Uses a sampled-sphere approximation for the torus corner:
        sample the torus as 8 spheres distributed around the corner,
        take the union of all sphere intervals.
        """
        O = np.asarray(ray_origin, dtype=np.float64)
        D = np.asarray(ray_direction, dtype=np.float64)
        d_norm = np.linalg.norm(D)
        if d_norm < 1e-20:
            return DexelColumn()
        D = D / d_norm

        R_tool = self.radius          # D / 2
        R_c = self.corner_radius      # corner radius
        R_flat = R_tool - R_c         # flat bottom radius
        FL = self.flute_length

        all_intervals: list[Tuple[float, float]] = []

        # 1. Cylinder section above the corner
        cyl = _ray_cylinder(O, D, R_tool, R_c, FL)
        all_intervals.extend(cyl)

        # 2. Flat bottom disc
        disc_t = _ray_disc(O, D, 0.0, R_flat)
        for t in disc_t:
            all_intervals.append((t, t))

        # 3. Torus approximation: sample the 90-degree arc as N spheres
        # The torus centre traces a circle of radius (R_tool - R_c) at z=R_c
        N = 8
        for k in range(N + 1):
            angle = (np.pi / 2.0) * k / N  # 0 to pi/2
            cx = (R_tool - R_c) * np.cos(angle)
            cz = R_c - R_c * np.sin(angle)
            centre = np.array([cx, 0.0, cz])
            # Rotate around Z to get full 360 coverage
            for m in range(N):
                phi = 2.0 * np.pi * m / N
                c_rot = np.array([
                    centre[0] * np.cos(phi),
                    centre[0] * np.sin(phi),
                    centre[2],
                ])
                sph = _ray_sphere(O, D, c_rot, R_c)
                all_intervals.extend(sph)

        return self._collect_intervals(all_intervals)

    def swept_volume_bbox(self, start_pose, end_pose) -> AABB:
        return self.bbox_at_origin()


# =========================================================================
#   Drill
# =========================================================================

class Drill(Tool):
    """Drill: cone tip + cylinder body.

    APT:  D > 0,  B = tip angle (typical 118 deg),  d = 0.

    Geometry:
    - Cone   from (0,0,0) with half-angle = B/2,  up to diameter = D
    - Cylinder  radius=D/2  from cone-height to FL
    """

    def __init__(self, diameter: float, tip_angle: float = 118.0,
                 flute_length: float = 50.0, length: float = 100.0, tip_diameter: float = 0.0):
        params = ToolParameters(
            diameter=diameter,
            length=length,
            flute_length=flute_length,
            tip_angle=tip_angle,
            tip_diameter=tip_diameter,
        )
        super().__init__(params)

    @property
    def cone_height(self) -> float:
        """Height of the conical tip from apex to full diameter."""
        return 0.0 if self.params.tip_angle == 180 else (self.radius - self.params.tip_diameter / 2) / np.tan(np.radians(self.params.tip_angle / 2.0))

    def ray_intersection(self, ray_origin: np.ndarray,
                         ray_direction: np.ndarray) -> DexelColumn:
        """Ray-drill intersection.

        1. Cylinder from z=h_cone to z=FL.
        2. Cone from z=0 to z=h_cone, half-angle = B/2.
        """
        O = np.asarray(ray_origin, dtype=np.float64)
        D = np.asarray(ray_direction, dtype=np.float64)
        d_norm = np.linalg.norm(D)
        if d_norm < 1e-20:
            return DexelColumn()
        D = D / d_norm

        R = self.radius
        FL = self.flute_length
        h_cone = self.cone_height
        half_angle = np.radians(self.params.tip_angle / 2.0)

        all_intervals: list[Tuple[float, float]] = []

        # 1. Cylinder
        cyl = _ray_cylinder(O, D, R, h_cone, FL)
        all_intervals.extend(cyl)

        # Intersect the finite point frustum directly. The radius at z is
        # r0 + z*tan(B/2). Clip the quadratic inequality to its actual Z slab.
        if h_cone > 1e-12:
            if abs(D[2]) <= 1e-12:
                lower, upper = (0.0, np.inf) if 0 <= O[2] <= h_cone else (1.0, 0.0)
            else:
                ends = sorted(((0 - O[2]) / D[2], (h_cone - O[2]) / D[2]))
                lower, upper = max(0.0, ends[0]), ends[1]
            if lower < upper:
                slope = np.tan(half_angle)
                r0 = self.params.tip_diameter / 2 + O[2] * slope
                a = D[0] ** 2 + D[1] ** 2 - (D[2] * slope) ** 2
                b = O[0] * D[0] + O[1] * D[1] - r0 * D[2] * slope
                c = O[0] ** 2 + O[1] ** 2 - r0 ** 2
                if abs(a) <= 1e-12:
                    if abs(b) <= 1e-12:
                        intervals = [(lower, upper)] if c <= 0 else []
                    else:
                        crossing = -c / (2 * b)
                        intervals = [(-np.inf, crossing)] if b > 0 else [(crossing, np.inf)]
                else:
                    roots = _solve_quadratic(a, b, c)
                    intervals = ([(-np.inf, np.inf)] if a < 0 else []) if roots is None else [roots] if a > 0 else [(-np.inf, roots[0]), (roots[1], np.inf)]
                all_intervals.extend((max(lower, first), min(upper, last)) for first, last in intervals
                    if max(lower, first) < min(upper, last))

        return self._collect_intervals(all_intervals)

    def swept_volume_bbox(self, start_pose, end_pose) -> AABB:
        return self.bbox_at_origin()

    def bbox_at_origin(self) -> AABB:
        R = self.radius
        return AABB(min=np.array([-R, -R, 0.0]),
                     max=np.array([R, R, self.flute_length]))


# =========================================================================
#   Chamfer Tool
# =========================================================================

class ChamferTool(Tool):
    """Chamfer tool: truncated cone (frustum).

    APT:  D > 0,  d > 0,  A = chamfer angle.

    Geometry:  radius varies linearly from d/2 at z=0 to D/2 at z=h.
    """

    @property
    def chamfer_height(self) -> float:
        """Height of the chamfer section."""
        return self.flute_length

    @property
    def tip_radius(self) -> float:
        return self.params.tip_diameter / 2.0

    def ray_intersection(self, ray_origin: np.ndarray,
                         ray_direction: np.ndarray) -> DexelColumn:
        """Ray-chamfer intersection: treat as a truncated cone.

        The chamfer has tip radius r0 at z=0 and full radius R at z=h.
        Half-angle = atan( (R-r0) / h ).
        """
        O = np.asarray(ray_origin, dtype=np.float64)
        D = np.asarray(ray_direction, dtype=np.float64)
        d_norm = np.linalg.norm(D)
        if d_norm < 1e-20:
            return DexelColumn()
        D = D / d_norm

        R = self.radius
        r0 = self.tip_radius
        h = self.chamfer_height

        if h < 1e-9:
            # Degenerate: flat disc
            disc_t = _ray_disc(O, D, 0.0, R)
            if len(disc_t) >= 2:
                return DexelColumn([DexelInterval(disc_t[0], disc_t[-1])])
            return DexelColumn()

        half_angle = np.arctan((R - r0) / h)

        # Shift the apex: the virtual apex is below z=0
        # r(z) = r0 + z * tan(half_angle)
        # r(0) = r0  => apex z_offset = -r0 / tan(half_angle)
        if half_angle > 0:
            apex_z = -r0 / np.tan(half_angle)
        else:
            # R = r0, straight cylinder
            apex_z = 0.0

        apex = np.array([0.0, 0.0, apex_z])
        total_height = h - apex_z  # height from virtual apex to top

        cone_intervals = _ray_cone(
            O, D, apex, half_angle, total_height, truncate_radius_min=r0
        )

        # Also check bottom disc at z=0 with tip radius
        if r0 > 0:
            disc_t = _ray_disc(O, D, 0.0, r0)
            for t in disc_t:
                cone_intervals.append((t, t))

        return self._collect_intervals(cone_intervals)

    def swept_volume_bbox(self, start_pose, end_pose) -> AABB:
        return self.bbox_at_origin()


# =========================================================================
#   Thread Mill
# =========================================================================

class ThreadMill(Tool):
    """Thread mill: approximated as a cylinder.

    Thread form detail is too fine for dexel resolution.
    Thread feasibility is validated by Tier-1 rules, not geometry simulation.
    """

    def ray_intersection(self, ray_origin: np.ndarray,
                         ray_direction: np.ndarray) -> DexelColumn:
        """Approximate thread mill as a plain cylinder of diameter D."""
        O = np.asarray(ray_origin, dtype=np.float64)
        D = np.asarray(ray_direction, dtype=np.float64)
        d_norm = np.linalg.norm(D)
        if d_norm < 1e-20:
            return DexelColumn()
        D = D / d_norm

        cyl = _ray_cylinder(O, D, self.radius, 0.0, self.flute_length)
        return self._collect_intervals(cyl)

    def swept_volume_bbox(self, start_pose, end_pose) -> AABB:
        return self.bbox_at_origin()


# =========================================================================
#   Dovetail Cutter
# =========================================================================

class DovetailCutter(Tool):
    """Dovetail cutter: reverse-taper cone.

    APT:  D > 0,  A = dovetail angle.

    Geometry:  radius decreases with depth (or increases, depending on
    reference).  Typically the narrow end is at the tip (z=0) and the
    wide end is at z=FL.

    Here we model the larger diameter at z=FL (cutting bottom) and the
    smaller diameter at z=0 (shank end of the tapered section).
    """

    @property
    def narrow_radius(self) -> float:
        return self.params.tip_diameter / 2.0 if self.params.tip_diameter > 0 else self.radius * 0.5

    def ray_intersection(self, ray_origin: np.ndarray,
                         ray_direction: np.ndarray) -> DexelColumn:
        """Dovetail cutter as a truncated cone.

        Narrow radius r_n at z=0, wide radius R at z=FL.
        (This is opposite to standard taper -- radius increases downward.)
        """
        O = np.asarray(ray_origin, dtype=np.float64)
        D = np.asarray(ray_direction, dtype=np.float64)
        d_norm = np.linalg.norm(D)
        if d_norm < 1e-20:
            return DexelColumn()
        D = D / d_norm

        R = self.radius
        r_n = self.narrow_radius
        h = self.flute_length

        if h < 1e-9:
            disc_t = _ray_disc(O, D, 0.0, R)
            if len(disc_t) >= 2:
                return DexelColumn([DexelInterval(disc_t[0], disc_t[-1])])
            return DexelColumn()

        half_angle = np.arctan((R - r_n) / h) if R > r_n else 0.0

        # Apex of the virtual cone is above the tool tip
        # r(z) = r_n + z*tan(half_angle) for z in [0, h]
        if half_angle > 0:
            apex_z = -r_n / np.tan(half_angle)
        else:
            apex_z = 0.0

        apex = np.array([0.0, 0.0, apex_z])
        total_height = h - apex_z

        cone_intervals = _ray_cone(
            O, D, apex, half_angle, total_height, truncate_radius_min=r_n
        )
        return self._collect_intervals(cone_intervals)

    def swept_volume_bbox(self, start_pose, end_pose) -> AABB:
        return self.bbox_at_origin()


# =========================================================================
#   Factory
# =========================================================================

def create_tool(tool_type: str, **kwargs) -> Tool:
    """Factory to create a tool by name.

    Args:
        tool_type: 'flat_endmill' | 'ball_endmill' | 'bull_nose' |
                   'drill' | 'chamfer' | 'thread_mill' | 'dovetail'
        **kwargs: Tool-specific parameters (diameter, flute_length, ...).

    Returns:
        A Tool subclass instance.
    """
    if tool_type == "ball_endmill":
        return BallEndmill(**kwargs)

    if tool_type == "drill":
        return Drill(**kwargs)

    tool_map: dict[str, type[Tool]] = {
        "flat_endmill": FlatEndmill,
        "bull_nose": BullNoseEndmill,
        "chamfer": ChamferTool,
        "thread_mill": ThreadMill,
        "dovetail": DovetailCutter,
    }
    if tool_type not in tool_map:
        raise ValueError(
            f"Unknown tool type '{tool_type}'. "
            f"Available: {list(tool_map.keys())} + ball_endmill, drill"
        )

    params = ToolParameters(**kwargs)
    return tool_map[tool_type](params)


# =========================================================================
#  Self-test
# =========================================================================

if __name__ == "__main__":
    print("=== Tool Ray-Intersection Self-Test ===\n")

    # ------------------------------------------------------------------
    # 1. BallEndmill -- central ray along axis
    # ------------------------------------------------------------------
    print("1. BallEndmill: central ray along +Z axis ...")
    tool = BallEndmill(diameter=10.0, flute_length=30.0)

    # Ray from below the tip straight up
    O = np.array([0.0, 0.0, -5.0])
    D = np.array([0.0, 0.0, 1.0])
    col = tool.ray_intersection(O, D)
    print(f"   Ray: O={O}, D={D}")
    print(f"   Intervals: {col}")
    assert col.count == 1, f"Expected 1 interval, got {col.count}"
    # Should enter at z=-5 + t*1 = -5+t... the sphere is centered at (0,0,5), radius 5
    # Lowest point of sphere is at z=0. Ray starts at z=-5.
    # Entry: sphere surface at z=0, t=5. Exit: cylinder top at z=30, t=35.
    iv = col[0]
    assert abs(iv.start - 5.0) < 1e-6, f"Expected entry at t=5.0, got {iv.start}"
    assert abs(iv.end - 35.0) < 1e-6, f"Expected exit at t=35.0, got {iv.end}"
    print("   PASSED\n")

    # ------------------------------------------------------------------
    # 2. BallEndmill -- ray from the side
    # ------------------------------------------------------------------
    print("2. BallEndmill: horizontal ray ...")
    O = np.array([-10.0, 0.0, 10.0])
    D = np.array([1.0, 0.0, 0.0])
    col = tool.ray_intersection(O, D)
    print(f"   Ray: O={O}, D={D}")
    print(f"   Intervals: {col}")
    # The cylinder is at z>=5, radius=5. At z=10, the cylinder has x in [-5, 5].
    # Entry: x=-5, t=5. Exit: x=5, t=15.
    assert col.count == 1, f"Expected 1 interval, got {col.count}"
    assert abs(col[0].start - 5.0) < 1e-6
    assert abs(col[0].end - 15.0) < 1e-6
    print("   PASSED\n")

    # ------------------------------------------------------------------
    # 3. BallEndmill -- ray that misses
    # ------------------------------------------------------------------
    print("3. BallEndmill: ray that misses ...")
    O = np.array([10.0, 10.0, 10.0])
    D = np.array([1.0, 1.0, 0.0])  # Normalised this would go further out
    D = D / np.linalg.norm(D)
    col = tool.ray_intersection(O, D)
    print(f"   Intervals: {col}")
    assert col.is_empty, f"Expected empty, got {col}"
    print("   PASSED\n")

    # ------------------------------------------------------------------
    # 4. FlatEndmill -- central ray
    # ------------------------------------------------------------------
    print("4. FlatEndmill: central ray along Z ...")
    tool2 = FlatEndmill(ToolParameters(diameter=10.0, flute_length=20.0))
    O = np.array([0.0, 0.0, -2.0])
    D = np.array([0.0, 0.0, 1.0])
    col = tool2.ray_intersection(O, D)
    print(f"   Intervals: {col}")
    assert col.count == 1
    # Entry at z=0 (bottom disc), t=2. Exit at z=20, t=22.
    assert abs(col[0].start - 2.0) < 1e-6
    assert abs(col[0].end - 22.0) < 1e-6
    print("   PASSED\n")

    # ------------------------------------------------------------------
    # 5. FlatEndmill -- horizontal ray below bottom
    # ------------------------------------------------------------------
    print("5. FlatEndmill: ray below bottom disc ...")
    O = np.array([0.0, 0.0, -1.0])
    D = np.array([1.0, 0.0, 0.0])
    col = tool2.ray_intersection(O, D)
    print(f"   Intervals: {col}")
    # The ray is in the plane z=-1, so it doesn't intersect the cylinder
    # (cylinder is at z>=0). It misses.
    # Actually, the ray at z=-1 with D=(1,0,0) misses the cylinder entirely
    # because Dz=0, so it stays at z=-1, below the cylinder.
    assert col.is_empty, f"Expected empty, got {col}"
    print("   PASSED\n")

    # ------------------------------------------------------------------
    # 6. Drill -- central ray
    # ------------------------------------------------------------------
    print("6. Drill: central ray along -Z (into the drill) ...")
    tool3 = Drill(diameter=10.0, tip_angle=118.0, flute_length=30.0)
    O = np.array([0.0, 0.0, 10.0])
    D = np.array([0.0, 0.0, -1.0])
    col = tool3.ray_intersection(O, D)
    print(f"   Drill cone height: {tool3.cone_height:.3f} mm")
    print(f"   Intervals: {col}")
    # t=0 at z=10. Entry into cylinder is at z=h_cone, exit at tip z=0.
    assert col.count >= 1, f"Expected at least 1 interval, got {col.count}"
    print("   PASSED\n")

    # ------------------------------------------------------------------
    # 7. Factory function
    # ------------------------------------------------------------------
    print("7. Factory: create_tool ...")
    t1 = create_tool("ball_endmill", diameter=12.0, flute_length=40.0)
    assert isinstance(t1, BallEndmill)
    assert abs(t1.diameter - 12.0) < 1e-9
    print(f"   Created: {t1}")

    t2 = create_tool("flat_endmill", diameter=8.0, flute_length=25.0)
    assert isinstance(t2, FlatEndmill)
    print(f"   Created: {t2}")

    t3 = create_tool("drill", diameter=6.0, tip_angle=118.0)
    assert isinstance(t3, Drill)
    print(f"   Created: {t3}")
    print("   PASSED\n")

    print("=== ALL TOOL TESTS PASSED ===")
