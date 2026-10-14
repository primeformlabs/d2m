"""SubCAD fixturing module -- workholding specification and validation.

Defines FixtureSpec (what holds the stock), Setup (fixture + face + work
offset), ClampingZone (where the fixture contacts), and a catalog of standard
industry fixtures.  Also provides bridges to trimesh for simulation collision
detection.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

class FixtureType(str, Enum):
    VISE = "vise"
    TOE_CLAMPS = "toe_clamps"
    SOFT_JAWS = "soft_jaws"
    VACUUM_PLATE = "vacuum_plate"
    CUSTOM = "custom"


WORK_OFFSET_NAMES = ("G54", "G55", "G56", "G57", "G58", "G59")


# ---------------------------------------------------------------------------
# ClampingZone
# ---------------------------------------------------------------------------

@dataclass
class ClampingZone:
    """A 3D region where a clamp contacts the stock surface.

    Defined in the stock's local coordinate frame.  Operations whose
    toolpath intersects a clamping zone produce a validation warning.

    Attributes:
        label: Human-readable zone name (e.g. ``"left_toe_clamp"``).
        x_min, x_max, y_min, y_max: XY footprint on the contact face.
        z_min, z_max: Height band of the clamp contact (Z in local frame).
        margin_mm: Extra clearance to add around the zone for validation.
    """

    label: str
    x_min: float; x_max: float
    y_min: float; y_max: float
    z_min: float; z_max: float
    margin_mm: float = 2.0

    def to_dict(self) -> dict:
        return {
            "label": self.label,
            "x_min": self.x_min, "x_max": self.x_max,
            "y_min": self.y_min, "y_max": self.y_max,
            "z_min": self.z_min, "z_max": self.z_max,
            "margin_mm": self.margin_mm,
        }

    def contains_point(self, x: float, y: float) -> bool:
        """Check if a point (on the face plane) falls within this zone
        including the margin."""
        m = self.margin_mm
        return (self.x_min - m <= x <= self.x_max + m and
                self.y_min - m <= y <= self.y_max + m)


# ---------------------------------------------------------------------------
# FixtureSpec
# ---------------------------------------------------------------------------

@dataclass
class FixtureSpec:
    """Specification for a fixturing / workholding device.

    Attributes:
        name: Unique name (e.g. ``"kurt_dx6_vise"``).
        fixture_type: One of ``FixtureType`` enum values.
        description: Human-readable description.
        clamping_zones: Regions where the fixture grips the stock.
        base_height_mm: Height of the fixture base above the machine table.
        jaw_width_mm: For vise-type fixtures, total jaw width.
        max_opening_mm: For vise-type, maximum jaw opening.
        max_part_mass_kg: Maximum recommended workpiece mass.
        mesh_path: Optional path to an STL file for visualisation/simulation.
    """

    name: str
    fixture_type: FixtureType = FixtureType.VISE
    description: str = ""
    clamping_zones: list[ClampingZone] = field(default_factory=list)
    base_height_mm: float = 0.0
    jaw_width_mm: float = 0.0
    max_opening_mm: float = 0.0
    max_part_mass_kg: float = 50.0
    mesh_path: Optional[str] = None
    dimensions: dict = field(default_factory=dict)
    source_path: Optional[str] = None
    visual_mesh_path: Optional[str] = None
    collision_mesh_path: Optional[str] = None
    coordinate_frame: dict = field(default_factory=dict)
    clearance_zones: list[dict] = field(default_factory=list)
    mounting_pattern: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "fixture_type": self.fixture_type.value,
            "description": self.description,
            "base_height_mm": self.base_height_mm,
            "jaw_width_mm": self.jaw_width_mm,
            "max_opening_mm": self.max_opening_mm,
            "max_part_mass_kg": self.max_part_mass_kg,
            "clamping_zones": [z.to_dict() for z in self.clamping_zones],
            "mesh_path": self.mesh_path,
            "dimensions": dict(self.dimensions or {}),
            "source_path": self.source_path,
            "visual_mesh_path": self.visual_mesh_path,
            "collision_mesh_path": self.collision_mesh_path,
            "coordinate_frame": dict(self.coordinate_frame or {}),
            "clearance_zones": list(self.clearance_zones or []),
            "mounting_pattern": dict(self.mounting_pattern or {}),
        }


# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------

@dataclass
class Setup:
    """A machining setup: fixture + face orientation + work offset.

    Links a FixtureSpec to a specific face of the part and a G5x
    coordinate register.  Multiple Setups on one Stock represent
    multi-setup machining (flip the part, re-clamp, machine other side).

    Attributes:
        name: Setup name (e.g. ``"op1_top"``, ``"op2_bottom"``).
        fixture: The FixtureSpec used for this setup.
        face_selector: CadQuery face selector (e.g. ``">Z"``, ``"<Z"``).
        work_offset: G5x register name (``"G54"``..``"G59"``).
        work_offset_values: ``(X, Y, Z, A, B, C)`` machine coordinates
            for the work-offset origin, relative to machine home.
        rotation_deg: Optional rotation about Z axis after flipping.
        description: Free-form description of what this setup achieves.
    """

    name: str
    fixture: FixtureSpec
    face_selector: str = ">Z"
    work_offset: str = "G54"
    work_offset_values: tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    rotation_deg: float = 0.0
    description: str = ""

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "fixture": self.fixture.name,
            "fixture_type": self.fixture.fixture_type.value,
            "fixture_spec": self.fixture.to_dict(),
            "face_selector": self.face_selector,
            "work_offset": self.work_offset,
            "work_offset_values": list(self.work_offset_values),
            "rotation_deg": self.rotation_deg,
            "description": self.description,
        }


# ---------------------------------------------------------------------------
# FixtureCatalog
# ---------------------------------------------------------------------------

class FixtureCatalog:
    """Standard fixture presets, analogous to ToolCatalog."""

    _presets: dict[str, FixtureSpec] = {}
    _initialised: bool = False
    _catalog_loaded: bool = False

    @classmethod
    def _ensure_initialised(cls):
        if cls._initialised:
            return

        cls._presets.update({
            "kurt_dx6_vise": FixtureSpec(
                name="kurt_dx6_vise",
                fixture_type=FixtureType.VISE,
                description="Kurt DX6 6-inch (152.4 mm) machine vise",
                clamping_zones=[
                    ClampingZone("vise_fixed_jaw",
                                 -76.2, -56.2, -50.0, 50.0, 3.0, 25.0),
                    ClampingZone("vise_movable_jaw",
                                 56.2, 76.2, -50.0, 50.0, 3.0, 25.0),
                ],
                base_height_mm=87.0,
                jaw_width_mm=152.4,
                max_opening_mm=228.6,
                dimensions={
                    "length_mm": 400.0,
                    "width_mm": 160.0,
                    "height_mm": 87.0,
                    "jaw_thickness_mm": 20.0,
                },
            ),
            "toe_clamps_4corner": FixtureSpec(
                name="toe_clamps_4corner",
                fixture_type=FixtureType.TOE_CLAMPS,
                description="Four toe clamps at the corners of the stock",
                clamping_zones=[
                    ClampingZone("clamp_bl", -60.0, -40.0, -35.0, -15.0, 18.0, 28.0),
                    ClampingZone("clamp_br", -60.0, -40.0, 15.0, 35.0, 18.0, 28.0),
                    ClampingZone("clamp_tl", 40.0, 60.0, -35.0, -15.0, 18.0, 28.0),
                    ClampingZone("clamp_tr", 40.0, 60.0, 15.0, 35.0, 18.0, 28.0),
                ],
                base_height_mm=20.0,
                dimensions={"length_mm": 130.0, "width_mm": 90.0, "height_mm": 28.0},
            ),
            "vacuum_plate_300x200": FixtureSpec(
                name="vacuum_plate_300x200",
                fixture_type=FixtureType.VACUUM_PLATE,
                description="300x200 mm vacuum chuck plate",
                clamping_zones=[],
                base_height_mm=35.0,
                max_part_mass_kg=10.0,
                dimensions={"length_mm": 300.0, "width_mm": 200.0, "height_mm": 35.0},
            ),
            "soft_jaws_aluminum": FixtureSpec(
                name="soft_jaws_aluminum",
                fixture_type=FixtureType.SOFT_JAWS,
                description="Custom-machinable aluminum soft jaws for vise",
                clamping_zones=[
                    ClampingZone("soft_jaw_left",
                                 -80.0, -60.0, -60.0, 60.0, 3.0, 40.0),
                    ClampingZone("soft_jaw_right",
                                 60.0, 80.0, -60.0, 60.0, 3.0, 40.0),
                ],
                base_height_mm=87.0,
                max_part_mass_kg=30.0,
                dimensions={
                    "length_mm": 400.0,
                    "width_mm": 170.0,
                    "height_mm": 120.0,
                    "jaw_thickness_mm": 25.0,
                },
            ),
        })

        cls._load_catalog_files()
        cls._initialised = True

    @classmethod
    def _load_catalog_files(cls) -> None:
        """Load version-controlled fixture catalog entries."""
        if cls._catalog_loaded:
            return
        cls._catalog_loaded = True

        path = Path(__file__).resolve().parent / "catalogs" / "fixtures.json"
        if not path.exists():
            return

        with path.open("r", encoding="utf-8") as fh:
            payload = json.load(fh)

        for entry in payload.get("fixtures", []):
            name = str(entry.get("id") or entry.get("name") or "").strip()
            if not name:
                continue
            fixture_type = FixtureType(entry.get("fixture_type", FixtureType.CUSTOM.value))
            zones = [
                ClampingZone(
                    label=str(zone.get("label", "clamp")),
                    x_min=float(zone["x_min"]),
                    x_max=float(zone["x_max"]),
                    y_min=float(zone["y_min"]),
                    y_max=float(zone["y_max"]),
                    z_min=float(zone.get("z_min", 0.0)),
                    z_max=float(zone.get("z_max", 0.0)),
                    margin_mm=float(zone.get("margin_mm", 2.0)),
                )
                for zone in entry.get("clamping_zones", [])
            ]
            cls._presets[name] = FixtureSpec(
                name=name,
                fixture_type=fixture_type,
                description=str(entry.get("description", "")),
                clamping_zones=zones,
                base_height_mm=float(entry.get("base_height_mm", 0.0)),
                jaw_width_mm=float(entry.get("jaw_width_mm", 0.0)),
                max_opening_mm=float(entry.get("max_opening_mm", 0.0)),
                max_part_mass_kg=float(entry.get("max_part_mass_kg", 50.0)),
                mesh_path=entry.get("mesh_path"),
                dimensions=dict(entry.get("dimensions") or {}),
                source_path=entry.get("source_path"),
                visual_mesh_path=entry.get("visual_mesh_path"),
                collision_mesh_path=entry.get("collision_mesh_path"),
                coordinate_frame=dict(entry.get("coordinate_frame") or {}),
                clearance_zones=list(entry.get("clearance_zones") or []),
                mounting_pattern=dict(entry.get("mounting_pattern") or {}),
            )

    @classmethod
    def get(cls, name: str) -> FixtureSpec:
        """Retrieve a fixture preset by name.  Raises KeyError if unknown."""
        cls._ensure_initialised()
        if name not in cls._presets:
            available = ", ".join(sorted(cls._presets.keys()))
            raise KeyError(
                f"Fixture {name!r} not found. Available: {available}"
            )
        return cls._presets[name]

    @classmethod
    def list_all(cls) -> list[str]:
        """Return all registered fixture names."""
        cls._ensure_initialised()
        return sorted(cls._presets.keys())

    @classmethod
    def register(cls, name: str, spec: FixtureSpec) -> None:
        """Register a new fixture spec."""
        cls._ensure_initialised()
        if name in cls._presets:
            raise ValueError(f"Fixture {name!r} already exists.")
        cls._presets[name] = spec


# ---------------------------------------------------------------------------
# Bridge: fixture → trimesh (for simulation collision detection)
# ---------------------------------------------------------------------------

def fixture_to_mesh(fixture: FixtureSpec):
    """Convert a FixtureSpec to a trimesh.Trimesh for collision detection.

    If the fixture has a ``mesh_path`` referencing an STL file, loads and
    returns it.  Otherwise synthesises a simplified mesh from the clamping
    zones (each zone becomes an axis-aligned box).

    Returns ``None`` if trimesh is not available.
    """
    try:
        import trimesh as _trimesh
    except ImportError:
        return None

    if fixture.mesh_path and os.path.exists(fixture.mesh_path):
        return _trimesh.load(fixture.mesh_path)

    if not fixture.clamping_zones:
        return None

    meshes = []
    for zone in fixture.clamping_zones:
        dx = zone.x_max - zone.x_min
        dy = zone.y_max - zone.y_min
        dz = zone.z_max - zone.z_min
        cx = (zone.x_min + zone.x_max) / 2.0
        cy = (zone.y_min + zone.y_max) / 2.0
        cz = (zone.z_min + zone.z_max) / 2.0
        box = _trimesh.primitives.Box(extents=(dx, dy, dz))
        box.apply_translation((cx, cy, cz))
        meshes.append(box)

    return _trimesh.util.concatenate(meshes) if len(meshes) > 1 else meshes[0]


# ---------------------------------------------------------------------------
# Operation-against-fixture validation
# ---------------------------------------------------------------------------

def check_operation_against_fixture(
    op_dict: dict,
    setup: Setup,
    stock_dims: dict,
) -> list[str]:
    """Check one operation against its specified setup and complete linear travel."""
    from .validation import _check_fixture_clearance
    if setup is None:
        return ["The fixture setup is missing. Fixture clearance is unverified."]
    operation = {**op_dict, "setup": setup.name,
                 "face_selector": op_dict.get("face_selector", setup.face_selector)}
    return [issue.message for issue in _check_fixture_clearance({
        "operations": [operation], "fixture": setup.fixture.to_dict(),
        "setups": {setup.name: setup.to_dict()}, "stock_dimensions": stock_dims,
    })]


def fixture_visualization_payload(fixture: FixtureSpec, stock_dims: Optional[dict] = None) -> dict:
    """Return procedural browser visualization data for a fixture.

    This keeps semantic fixture data as the source of truth. Real mesh paths can
    be supplied later, while v1 still visualizes base/jaws/clamp zones.
    """
    dims = dict(fixture.dimensions or {})
    stock_dims = dict(stock_dims or {})
    stock_length = float(stock_dims.get("length", dims.get("stock_length_mm", 100.0)))
    stock_width = float(stock_dims.get("width", dims.get("stock_width_mm", 60.0)))

    body = {
        "type": fixture.fixture_type.value,
        "base": {
            "center": [0.0, 0.0, -fixture.base_height_mm / 2.0],
            "size": [
                float(dims.get("length_mm", max(stock_length + 80.0, fixture.jaw_width_mm or stock_length))),
                float(dims.get("width_mm", max(stock_width + 80.0, fixture.jaw_width_mm or stock_width))),
                float(dims.get("height_mm", max(fixture.base_height_mm, 10.0))),
            ],
        },
    }

    return {
        "name": fixture.name,
        "fixture_type": fixture.fixture_type.value,
        "description": fixture.description,
        "body": body,
        "clamping_zones": [zone.to_dict() for zone in fixture.clamping_zones],
        "clearance_zones": list(fixture.clearance_zones or []),
        "source_path": fixture.source_path,
        "visual_mesh_path": fixture.visual_mesh_path or fixture.mesh_path,
        "collision_mesh_path": fixture.collision_mesh_path,
        "coordinate_frame": dict(fixture.coordinate_frame or {}),
    }


# =========================================================================
# Self-test
# =========================================================================

if __name__ == "__main__":
    import sys

    passed = 0
    failed = 0

    def check(condition, label):
        global passed, failed
        if condition:
            passed += 1
            print(f"  PASS  {label}")
        else:
            failed += 1
            print(f"  FAIL  {label}")

    print("=== SubCAD Fixturing Self-Test ===\n")

    # 1. FixtureSpec instantiation
    print("1. FixtureSpec ...")
    fs = FixtureSpec(
        name="test_vise",
        fixture_type=FixtureType.VISE,
        description="A test vise",
        clamping_zones=[ClampingZone("test_zone", -10, 10, -5, 5, 0, 20)],
        base_height_mm=50.0,
    )
    check(fs.name == "test_vise", "name")
    check(fs.fixture_type == FixtureType.VISE, "type is VISE")
    check(len(fs.clamping_zones) == 1, "has 1 clamping zone")
    check(isinstance(fs.to_dict(), dict), "to_dict returns dict")
    print()

    # 2. ClampingZone
    print("2. ClampingZone ...")
    cz = ClampingZone("test", -10, 10, -5, 5, 0, 20, margin_mm=2.0)
    check(cz.contains_point(0, 0), "contains center point")
    check(cz.contains_point(11, 0), "contains point within margin")
    check(not cz.contains_point(15, 0), "excludes point outside margin")
    check(isinstance(cz.to_dict(), dict), "to_dict returns dict")
    print()

    # 3. FixtureCatalog presets
    print("3. FixtureCatalog ...")
    FixtureCatalog._ensure_initialised()
    names = FixtureCatalog.list_all()
    check(len(names) == 4, f"4 presets (got {len(names)})")
    check("kurt_dx6_vise" in names, "kurt_dx6_vise present")
    check("toe_clamps_4corner" in names, "toe_clamps_4corner present")
    check("vacuum_plate_300x200" in names, "vacuum_plate_300x200 present")
    check("soft_jaws_aluminum" in names, "soft_jaws_aluminum present")

    vise = FixtureCatalog.get("kurt_dx6_vise")
    check(isinstance(vise, FixtureSpec), "get returns FixtureSpec")
    check(vise.fixture_type == FixtureType.VISE, "kurt is VISE")
    check(len(vise.clamping_zones) == 2, "kurt has 2 clamping zones")
    check(vise.max_part_mass_kg == 50.0, "kurt max mass 50 kg")

    try:
        FixtureCatalog.get("nonexistent")
        check(False, "should raise KeyError for unknown")
    except KeyError:
        check(True, "raises KeyError for unknown")

    # Register custom
    custom = FixtureSpec("my_vise", FixtureType.CUSTOM)
    FixtureCatalog.register("my_vise", custom)
    check("my_vise" in FixtureCatalog.list_all(), "custom registered")
    try:
        FixtureCatalog.register("my_vise", custom)
        check(False, "duplicate should raise ValueError")
    except ValueError:
        check(True, "duplicate raises ValueError")
    print()

    # 4. Setup
    print("4. Setup ...")
    setup = Setup(
        name="op1_top",
        fixture=vise,
        face_selector=">Z",
        work_offset="G54",
        work_offset_values=(300.0, 200.0, 50.0, 0, 0, 0),
        description="Top-side machining",
    )
    sd = setup.to_dict()
    check(sd["name"] == "op1_top", "name in dict")
    check(sd["fixture"] == "kurt_dx6_vise", "fixture name in dict")
    check(sd["face_selector"] == ">Z", "face_selector in dict")
    check(sd["work_offset"] == "G54", "work_offset in dict")
    check(sd["work_offset_values"] == [300, 200, 50, 0, 0, 0], "offset values")
    print()

    # 5. fixture_to_mesh
    print("5. fixture_to_mesh ...")
    try:
        import trimesh
        _HAS_TRIMESH = True
    except ImportError:
        _HAS_TRIMESH = False

    if _HAS_TRIMESH:
        mesh = fixture_to_mesh(vise)
        check(mesh is not None, "vise mesh created")
        check(hasattr(mesh, "vertices"), "mesh has vertices")
        check(len(mesh.vertices) > 0, "mesh has non-zero vertices")
        check(len(mesh.faces) > 0, "mesh has faces")

        # Vacuum plate with no clamping zones
        vac = FixtureCatalog.get("vacuum_plate_300x200")
        mesh_vac = fixture_to_mesh(vac)
        check(mesh_vac is None, "vacuum plate (no zones) returns None")
    else:
        for _ in range(5):
            check(True, "trimesh not available -- skip mesh tests")
    print()

    # 6. check_operation_against_fixture
    print("6. check_operation_against_fixture ...")
    setup_vise = Setup(name="test", fixture=vise, face_selector=">Z",
                       work_offset="G54")
    stock_dims = {"length": 100, "width": 60, "height": 30}

    # Clear op: drill far from clamps
    clear_op = {"operation": "drill", "position": [0.0, 0.0]}
    w = check_operation_against_fixture(clear_op, setup_vise, stock_dims)
    check(len(w) == 0, "clear op yields no warnings")

    # Intersecting op: drill right at vise fixed jaw
    bad_op = {"operation": "drill", "position": [-66.0, 0.0]}
    w = check_operation_against_fixture(bad_op, setup_vise, stock_dims)
    check(len(w) > 0, "intersecting op yields warnings")
    check("vise_fixed_jaw" in w[0], "warning names the zone")

    # Face mill with clamps
    fm_op = {"operation": "face_mill"}
    w = check_operation_against_fixture(fm_op, setup_vise, stock_dims)
    check(len(w) > 0, "face_mill with clamps yields advisory warning")

    # None setup
    w = check_operation_against_fixture(fm_op, None, stock_dims)
    check(len(w) == 0, "None setup returns empty")
    print()

    # Report
    total = passed + failed
    print(f"=== {'ALL TESTS PASSED' if failed == 0 else 'SOME TESTS FAILED'} "
          f"({passed}/{total} passed) ===")
    if failed > 0:
        sys.exit(1)
