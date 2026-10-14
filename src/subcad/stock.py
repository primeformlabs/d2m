"""SubCAD Stock class -- the main fluent entry point.

The Stock class wraps a CadQuery B-Rep shape, a material key, and an
accumulating ProcessPlan. Every machining operation returns a new Stock
instance (immutable pattern, like CadQuery), so chains are composable.

Usage:
    part = (Stock.rectangular(100, 50, 20, material="aluminum_6061")
        .face_mill(depth=1.0)
        .pocket(width=30, length=20, depth=5, corner_radius=2)
        .drill(diameter=6, cx=25, cy=10, depth=15)
        .chamfer(width=0.5)
    )
    part.to_step("output.step")
    print(part.process_plan())
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

from .geometry import (
    create_rectangular_stock,
    create_cylindrical_stock,
    export_step,
    export_stl,
    to_mesh,
    compute_bounding_box,
    compute_volume,
    import_step,
    _HAS_CADQUERY,
)
from .process_plan import ProcessPlan
from .profiles import profile_span_yx
from .tool_library import ToolSpec
from .tool_selection import validate_inventory_tool


def _profile_bounds_for_stock(profile) -> tuple[float, float]:
    """Return (width_y, length_x) from a lightweight profile description."""
    return profile_span_yx(profile)


def _profile_through_depth(stock, face_selector: str) -> float:
    if face_selector not in {">X", "<X", ">Y", "<Y", ">Z", "<Z"}:
        raise ValueError("The profile requires one of the six cardinal setup faces.")
    axis = face_selector[-1].lower()
    bounds = stock.bounding_box
    return bounds[f"{axis}_max"] - bounds[f"{axis}_min"]


def _profiles_bounds_for_stock(profiles) -> tuple[float, float]:
    """Return (width_y, length_x) spanning multiple lightweight profiles."""
    bounds = []
    from .profiles import profile_bounds_xy

    for profile in profiles or []:
        bounds.append(profile_bounds_xy(profile))
    if not bounds:
        return 10.0, 10.0
    xmin = min(item[0] for item in bounds)
    ymin = min(item[1] for item in bounds)
    xmax = max(item[2] for item in bounds)
    ymax = max(item[3] for item in bounds)
    return ymax - ymin, xmax - xmin


def _place_profile(profile, cx: Optional[float] = None, cy: Optional[float] = None):
    """Return a profile shifted/centered by optional fluent placement kwargs."""
    if cx is None and cy is None:
        return profile
    dx = float(cx or 0.0)
    dy = float(cy or 0.0)
    if isinstance(profile, dict):
        placed = dict(profile)
        point_key = next(
            (key for key in ("points", "vertices", "polyline", "control_points") if placed.get(key)),
            None,
        )
        if point_key:
            placed[point_key] = [(float(point[0]) + dx, float(point[1]) + dy) for point in placed[point_key]]
            return placed
        if cx is not None:
            placed["cx"] = dx
        if cy is not None:
            placed["cy"] = dy
        if placed.get("center") is not None:
            center = list(placed["center"])
            if cx is not None:
                center[0] = dx
            if cy is not None:
                center[1] = dy
            placed["center"] = center
        return placed
    if isinstance(profile, (list, tuple)):
        return [(float(point[0]) + dx, float(point[1]) + dy) for point in profile]
    return profile


def _as_pair(value, *, name: str) -> tuple[float, float]:
    """Return a numeric (x, y) pair from a scalar or two-value tuple/list."""
    if value is None:
        raise ValueError(f"{name} is required")
    if isinstance(value, (int, float)):
        scalar = float(value)
        return scalar, scalar
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return float(value[0]), float(value[1])
    raise ValueError(f"{name} must be a number or a two-value (x, y) pair")


@dataclass
class Stock:
    """An in-process workpiece with an accumulating manufacturing plan.

    Create instances via factory constructors:
      - Stock.rectangular(length, width, height, material)
      - Stock.cylindrical(diameter, height, material)
      - Stock.from_step(path, material)

    Each machining method returns a **new** Stock, so chains are safe:
        s2 = s1.face_mill(1.0).pocket(30, 20, 5)
        # s1 is unchanged, s2 has both operations applied.
    """

    _shape: object                # CadQuery Workplane / Solid
    _material: str                # e.g. "aluminum_6061"
    _plan: ProcessPlan = field(default_factory=ProcessPlan)
    _stock_dims: dict = field(default_factory=dict)
    _next_op_number: int = 1
    _fixture: Optional["FixtureSpec"] = None
    _setups: dict = field(default_factory=dict)  # name -> Setup
    _active_setup: Optional[str] = None
    _state_history: list = field(default_factory=list)

    # -------------------------------------------------------------------
    #  Internal helpers
    # -------------------------------------------------------------------

    @classmethod
    def _from_parts(
        cls,
        shape: object,
        material: str,
        stock_dims: dict,
        plan: ProcessPlan,
        next_op_number: int,
        fixture=None,
        setups=None,
        active_setup=None,
        state_history=None,
    ) -> "Stock":
        """Construct directly from components."""
        obj = object.__new__(cls)
        obj._shape = shape
        obj._material = material
        obj._plan = plan
        obj._stock_dims = stock_dims
        obj._next_op_number = next_op_number
        obj._fixture = fixture
        obj._setups = setups if setups is not None else {}
        obj._active_setup = active_setup
        obj._state_history = state_history if state_history is not None else []
        return obj

    def _apply_op(self, op) -> "Stock":
        """Apply a machining operation: modify shape, record in plan, return new Stock."""
        validate_inventory_tool(getattr(op, "tool", None), depth=getattr(op, "depth", 0.0))
        if getattr(op, "sequence_number", 0) == 0:
            op.sequence_number = self._next_op_number

        # Look up active setup and apply its face_selector to the operation
        active_setup = self._active_setup
        if active_setup is not None and active_setup in self._setups:
            setup = self._setups[active_setup]
            if hasattr(op, "face_selector"):
                op.face_selector = setup.face_selector

        new_shape = op.apply(self._shape)
        state_history = list(self._state_history)
        if not state_history:
            state_history.append({
                "sequence_number": 0,
                "operation": "initial_stock",
                "label": "Initial stock",
                "shape": self._shape,
            })
        passes = getattr(op, "_material_pass_states", None)
        if passes:
            state_history.extend({**state, "sequence_number": op.sequence_number,
                "operation": getattr(op, "operation_name", op.__class__.__name__)} for state in passes)
        else:
            state_history.append({
                "sequence_number": op.sequence_number,
                "operation": getattr(op, "operation_name", op.__class__.__name__),
                "label": f"After OP {op.sequence_number}",
                "shape": new_shape,
            })

        # Serialize fixture and setups for the plan. Runtime FixtureSpec/Setup
        # objects win, but context-only setup intent must survive operation chains.
        fixture_dict = (
            self._fixture.to_dict()
            if self._fixture
            else (dict(self._plan.fixture) if self._plan.fixture else None)
        )
        if self._setups:
            setups_dict = {name: s.to_dict() for name, s in self._setups.items()}
            work_offsets_dict = {}
            for name, s in self._setups.items():
                wo = getattr(s, "work_offset", "G54")
                wov = getattr(s, "work_offset_values", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                work_offsets_dict[wo] = list(wov)
        else:
            setups_dict = dict(self._plan.setups)
            work_offsets_dict = dict(self._plan.work_offsets)

        new_plan = ProcessPlan(
            material=self._material,
            stock_dimensions=dict(self._stock_dims),
            operations=list(self._plan.operations),
            tools_used=list(self._plan.tools_used),
            fixture=fixture_dict,
            setups=setups_dict,
            work_offsets=work_offsets_dict,
            machine=dict(self._plan.machine) if self._plan.machine else None,
            controller=dict(self._plan.controller) if self._plan.controller else None,
            setup_intent=dict(self._plan.setup_intent) if self._plan.setup_intent else None,
            tooling=dict(self._plan.tooling) if self._plan.tooling else None,
            quality=dict(self._plan.quality) if self._plan.quality else None,
            collision_policy=(
                dict(self._plan.collision_policy)
                if self._plan.collision_policy else None
            ),
        )

        op_dict = op.to_dict()
        op_dict["sequence_number"] = op.sequence_number
        tool_selection = getattr(op, "tool_selection", None)
        if tool_selection is not None and hasattr(tool_selection, "to_dict"):
            op_dict["tool_selection"] = tool_selection.to_dict()

        if hasattr(op, "to_toolpath"):
            try:
                toolpath = op.to_toolpath()
            except Exception as error:
                toolpath = []
                op_dict["toolpath_status"] = "unavailable"
                op_dict["toolpath_reason"] = str(error)
            if toolpath:
                serialized_toolpath = _serialize_toolpath(toolpath)
                if serialized_toolpath:
                    op_dict["toolpath"] = serialized_toolpath
                    metadata = (
                        serialized_toolpath.get("metadata", {})
                        if isinstance(serialized_toolpath, dict)
                        else {}
                    )
                    if isinstance(metadata, dict):
                        if "pass_plan" in metadata and "pass_plan" not in op_dict:
                            op_dict["pass_plan"] = metadata["pass_plan"]
                        if "tool_selection" in metadata and "tool_selection" not in op_dict:
                            op_dict["tool_selection"] = metadata["tool_selection"]

        # Attach setup and face_selector info to the op record
        if active_setup is not None and active_setup in self._setups:
            op_dict["setup"] = active_setup
            op_dict["face_selector"] = self._setups[active_setup].face_selector
        elif self._plan.setup_intent and len(setups_dict) == 1:
            setup_name, setup = next(iter(setups_dict.items()))
            op_dict.setdefault("setup", setup_name)
            if isinstance(setup, dict):
                op_dict.setdefault(
                    "face_selector",
                    setup.get(
                        "face_selector",
                        self._plan.setup_intent.get("primary_face", ">Z"),
                    ),
                )

        new_plan.add_operation(op_dict)

        return Stock._from_parts(
            new_shape,
            self._material,
            self._stock_dims,
            new_plan,
            self._next_op_number + 1,
            fixture=self._fixture,
            setups=dict(self._setups),
            active_setup=self._active_setup,
            state_history=state_history,
        )

    def _rectangular_xy_limits(self) -> tuple[float, float, float, float]:
        """Return left, right, bottom, top stock limits for top-face XY work."""
        try:
            length = float(self._stock_dims["length"])
            width = float(self._stock_dims["width"])
        except KeyError as exc:
            raise ValueError("natural XY placement requires rectangular stock") from exc
        return -length / 2.0, length / 2.0, -width / 2.0, width / 2.0

    def resolve_xy(
        self,
        *,
        x: Optional[float] = None,
        y: Optional[float] = None,
        from_left: Optional[float] = None,
        from_right: Optional[float] = None,
        from_bottom: Optional[float] = None,
        from_top: Optional[float] = None,
        center_x: bool = False,
        center_y: bool = False,
        dx: float = 0.0,
        dy: float = 0.0,
    ) -> tuple[float, float]:
        """Resolve natural top-face placement references into explicit ``cx, cy``.

        Rectangular stock is centered at the origin, so ``from_left=10`` means
        ``cx = -length / 2 + 10`` and ``from_top=8`` means
        ``cy = width / 2 - 8``. Use either one X reference and one Y reference,
        or ``center_x`` / ``center_y`` for centerline placement.
        """
        left, right, bottom, top = self._rectangular_xy_limits()

        x_refs = [
            x is not None,
            from_left is not None,
            from_right is not None,
            bool(center_x),
        ]
        y_refs = [
            y is not None,
            from_bottom is not None,
            from_top is not None,
            bool(center_y),
        ]
        if sum(x_refs) != 1:
            raise ValueError("specify exactly one X reference")
        if sum(y_refs) != 1:
            raise ValueError("specify exactly one Y reference")

        if x is not None:
            cx = float(x)
        elif from_left is not None:
            cx = left + float(from_left)
        elif from_right is not None:
            cx = right - float(from_right)
        else:
            cx = 0.0

        if y is not None:
            cy = float(y)
        elif from_bottom is not None:
            cy = bottom + float(from_bottom)
        elif from_top is not None:
            cy = top - float(from_top)
        else:
            cy = 0.0

        return cx + float(dx), cy + float(dy)

    def _mirror_xy_points(
        self,
        points: list[tuple[float, float]],
        *,
        mirror_x: bool = False,
        mirror_y: bool = False,
    ) -> list[tuple[float, float]]:
        """Mirror points across stock centerlines and remove duplicates."""
        mirrored: list[tuple[float, float]] = []
        for cx, cy in points:
            x_values = [cx, -cx] if mirror_x else [cx]
            y_values = [cy, -cy] if mirror_y else [cy]
            for x_value in x_values:
                for y_value in y_values:
                    point = (float(x_value), float(y_value))
                    if not any(
                        abs(point[0] - old[0]) < 1e-9 and abs(point[1] - old[1]) < 1e-9
                        for old in mirrored
                    ):
                        mirrored.append(point)
        return mirrored

    def _plan_with_context(self, **updates) -> ProcessPlan:
        """Return a copy of the current plan with selected context replaced."""
        context = {
            "machine": dict(self._plan.machine) if self._plan.machine else None,
            "controller": dict(self._plan.controller) if self._plan.controller else None,
            "setup_intent": dict(self._plan.setup_intent) if self._plan.setup_intent else None,
            "tooling": dict(self._plan.tooling) if self._plan.tooling else None,
            "quality": dict(self._plan.quality) if self._plan.quality else None,
            "collision_policy": (
                dict(self._plan.collision_policy)
                if self._plan.collision_policy else None
            ),
        }
        fixture = updates.pop(
            "fixture",
            dict(self._plan.fixture) if self._plan.fixture else None,
        )
        setups = updates.pop("setups", dict(self._plan.setups))
        work_offsets = updates.pop("work_offsets", dict(self._plan.work_offsets))
        context.update(updates)
        return ProcessPlan(
            material=self._material,
            stock_dimensions=dict(self._stock_dims),
            operations=list(self._plan.operations),
            tools_used=list(self._plan.tools_used),
            schema_version=self._plan.schema_version,
            fixture=fixture,
            setups=setups,
            work_offsets=work_offsets,
            **context,
        )

    def _with_plan_context(self, **updates) -> "Stock":
        """Return a new Stock with copied geometry and updated plan context."""
        return Stock._from_parts(
            self._shape,
            self._material,
            self._stock_dims,
            self._plan_with_context(**updates),
            self._next_op_number,
            fixture=self._fixture,
            setups=dict(self._setups),
            active_setup=self._active_setup,
            state_history=list(self._state_history),
        )

    def with_machine(self, machine) -> "Stock":
        """Bind this process plan to a machine definition for validation."""
        from .machine_context import load_machine_context

        context = load_machine_context(machine).to_dict()
        return self._with_plan_context(machine=context)

    def with_controller(self, controller) -> "Stock":
        """Bind this process plan to a controller/post definition."""
        from .controller_context import load_controller_context

        context = load_controller_context(controller).to_dict()
        return self._with_plan_context(controller=context)

    def on_controller(self, controller) -> "Stock":
        """Alias for :meth:`with_controller`."""
        return self.with_controller(controller)

    def with_setup_intent(self, setup_intent) -> "Stock":
        """Attach datum, orientation, workholding, and access intent."""
        from .setup_context import load_setup_context

        setup_context = load_setup_context(setup_intent)
        context = setup_context.to_dict()
        metadata = setup_context.to_plan_metadata()
        setups = dict(self._plan.setups)
        setups.update(metadata.get("setups") or {})
        work_offsets = dict(self._plan.work_offsets)
        work_offsets.update(metadata.get("work_offsets") or {})
        fixture = metadata.get("fixture")
        if fixture is None:
            fixture = dict(self._plan.fixture) if self._plan.fixture else None
        return self._with_plan_context(
            setup_intent=context,
            fixture=fixture,
            setups=setups,
            work_offsets=work_offsets,
        )

    def with_tooling(self, tooling) -> "Stock":
        """Attach available tool assemblies and magazine context."""
        from .tooling_context import load_tooling_context

        context = load_tooling_context(tooling).to_dict()
        return self._with_plan_context(tooling=context)

    def with_quality(self, quality) -> "Stock":
        """Attach tolerance, inspection, and surface-finish requirements."""
        from .quality_context import load_quality_context

        context = load_quality_context(quality).to_dict()
        return self._with_plan_context(quality=context)

    def with_inspection(self, inspection) -> "Stock":
        """Attach inspection requirements while preserving existing quality data."""
        quality = dict(self._plan.quality or {})
        quality["inspection"] = inspection
        return self.with_quality(quality)

    def with_collision_policy(self, policy) -> "Stock":
        """Attach collision and clearance validation policy."""
        from .quality_context import load_collision_policy

        context = load_collision_policy(policy)
        return self._with_plan_context(collision_policy=context)

    def require_collision_check(
        self,
        *,
        clearance_mm: float = 2.0,
        holder_check: bool = True,
        fixture_check: bool = True,
        toolpath_required: bool = True,
    ) -> "Stock":
        """Require collision validation before shop-floor handoff."""
        policy = dict(self._plan.collision_policy or {})
        policy.update({
            "enabled": True,
            "required": True,
            "min_clearance_mm": float(clearance_mm),
            "holder_check": bool(holder_check),
            "fixture_check": bool(fixture_check),
            "toolpath_required": bool(toolpath_required),
        })
        return self.with_collision_policy(policy)

    def with_process_context(
        self,
        *,
        machine=None,
        controller=None,
        setup_intent=None,
        tooling=None,
        quality=None,
        collision_policy=None,
    ) -> "Stock":
        """Attach several manufacturing contexts in one fluent step."""
        result = self
        if machine is not None:
            result = result.with_machine(machine)
        if controller is not None:
            result = result.with_controller(controller)
        if setup_intent is not None:
            result = result.with_setup_intent(setup_intent)
        if tooling is not None:
            result = result.with_tooling(tooling)
        if quality is not None:
            result = result.with_quality(quality)
        if collision_policy is not None:
            result = result.with_collision_policy(collision_policy)
        return result

    def with_process_constraints(self, **kwargs) -> "Stock":
        """Alias for :meth:`with_process_context`."""
        return self.with_process_context(**kwargs)

    def with_manufacturing_context(self, **kwargs) -> "Stock":
        """Alias for :meth:`with_process_context`."""
        return self.with_process_context(**kwargs)

    def on_machine(self, machine) -> "Stock":
        """Alias for :meth:`with_machine`."""
        return self.with_machine(machine)

    # -------------------------------------------------------------------
    #  Factory constructors
    # -------------------------------------------------------------------

    @classmethod
    def rectangular(
        cls,
        length: float,
        width: float,
        height: float,
        material: str = "aluminum_6061",
    ) -> "Stock":
        """Create rectangular prismatic stock.

        Args:
            length: X-axis dimension (mm).
            width:  Y-axis dimension (mm).
            height: Z-axis dimension (mm).
            material: Material key.
        """
        shape = create_rectangular_stock(length, width, height)
        dims = {"length": length, "width": width, "height": height}
        return cls(
            _shape=shape,
            _material=material,
            _plan=ProcessPlan(material=material, stock_dimensions=dims),
            _stock_dims=dims,
        )

    @classmethod
    def sheet(
        cls,
        length: float,
        width: float,
        thickness: float,
        material: str = "steel_a36",
    ) -> "Stock":
        """Create flat sheet stock for laser cutting and press-brake forming."""
        shape = create_rectangular_stock(length, width, thickness)
        dims = {
            "length": length,
            "width": width,
            "height": thickness,
            "thickness": thickness,
            "stock_form": "sheet",
            "process_family": "sheet_metal",
        }
        return cls(
            _shape=shape,
            _material=material,
            _plan=ProcessPlan(material=material, stock_dimensions=dims),
            _stock_dims=dims,
        )

    @classmethod
    def cylindrical(
        cls,
        diameter: float,
        height: float,
        material: str = "aluminum_6061",
    ) -> "Stock":
        """Create cylindrical stock from round bar."""
        shape = create_cylindrical_stock(diameter, height)
        dims = {"diameter": diameter, "height": height}
        return cls(
            _shape=shape,
            _material=material,
            _plan=ProcessPlan(material=material, stock_dimensions=dims),
            _stock_dims=dims,
        )

    @classmethod
    def round_bar(cls, length: float, diameter: float, material: str = "aluminum_6061", *,
                  axis: str = "Z", origin: Tuple[float, float, float] = (0, 0, 0)) -> "Stock":
        """Create actual round stock in the spindle's part coordinate frame."""
        from .turning_geometry import finite, frame, from_spindle

        length = finite(length, "stock length", positive=True)
        diameter = finite(diameter, "stock diameter", positive=True)
        axis, origin = frame(axis, origin)
        shape = from_spindle(create_cylindrical_stock(diameter, length), axis, origin)
        dims = {"diameter": diameter, "height": length, "length": length, "stock_form": "round_bar",
            "axis": axis, "spindle_origin_mm": list(origin)}
        return cls(_shape=shape, _material=material, _plan=ProcessPlan(material=material, stock_dimensions=dims),
            _stock_dims=dims)

    @classmethod
    def from_step(
        cls,
        path: str,
        material: str = "aluminum_6061",
        stock_oversize: float = 5.0,
    ) -> "Stock":
        """Import a STEP file and wrap in an oversize stock billet."""
        imported = import_step(path)
        bbox = compute_bounding_box(imported)
        stock_l = bbox["length"] + 2 * stock_oversize
        stock_w = bbox["width"] + 2 * stock_oversize
        stock_h = bbox["height"] + stock_oversize

        shape = create_rectangular_stock(stock_l, stock_w, stock_h)
        dims = {"length": stock_l, "width": stock_w, "height": stock_h}
        return cls(
            _shape=shape,
            _material=material,
            _plan=ProcessPlan(material=material, stock_dimensions=dims),
            _stock_dims=dims,
        )

    # -------------------------------------------------------------------
    #  Machining operations  (each returns a new Stock)
    # -------------------------------------------------------------------

    def face_mill(
        self,
        depth: float = 1.0,
        *,
        tool: Optional[ToolSpec] = None,
        stepover: Optional[float] = None,
        face_selector: str = ">Z",
    ) -> "Stock":
        """Face mill the top surface.

        Args:
            depth: Material to remove from top face (mm).
            tool: ToolSpec or None (auto-selects).
            stepover: Radial engagement per pass (default 0.7 * tool_dia).
        """
        from .operations import FaceMillOp

        op = FaceMillOp(
            depth=depth,
            stepover=stepover,
            tool=tool,
            material=self._material,
            face_selector=face_selector,
        )
        op.sequence_number = self._next_op_number
        return self._apply_op(op)

    def pocket(
        self,
        width: float,
        length: float,
        depth: float,
        *,
        corner_radius: float = 0.0,
        cx: float = 0.0,
        cy: float = 0.0,
        base_height: Optional[float] = None,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a rectangular pocket.

        If corner_radius < tool.radius, a finish pass is automatically added.

        Args:
            width: Pocket width along Y (mm).
            length: Pocket length along X (mm).
            depth: Total pocket depth (mm).
            corner_radius: Internal corner radius (mm).
            cx, cy: Center position on top face (mm).
            tool: ToolSpec or None (auto-selects ~80% of width).
        """
        from .operations import PocketOp

        op = PocketOp(
            cx=cx,
            cy=cy,
            width=width,
            length=length,
            depth=depth,
            corner_radius=corner_radius,
            base_height=base_height,
            face_selector=face_selector,
            tool=tool,
            material=self._material,
        )
        op.sequence_number = self._next_op_number
        result = self._apply_op(op)

        # If PocketOp auto-created a finish op, apply it too
        finish = getattr(op, "finish_op", None)
        if finish is not None:
            finish.face_selector = op.face_selector
            finish.sequence_number = result._next_op_number
            result = result._apply_op(finish)

        return result

    def pocket_at(
        self,
        width: float,
        length: float,
        depth: float,
        *,
        x: Optional[float] = None,
        y: Optional[float] = None,
        from_left: Optional[float] = None,
        from_right: Optional[float] = None,
        from_bottom: Optional[float] = None,
        from_top: Optional[float] = None,
        center_x: bool = False,
        center_y: bool = False,
        dx: float = 0.0,
        dy: float = 0.0,
        corner_radius: float = 0.0,
        base_height: Optional[float] = None,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a rectangular pocket using natural top-face XY references."""
        cx, cy = self.resolve_xy(
            x=x,
            y=y,
            from_left=from_left,
            from_right=from_right,
            from_bottom=from_bottom,
            from_top=from_top,
            center_x=center_x,
            center_y=center_y,
            dx=dx,
            dy=dy,
        )
        return self.pocket(
            width,
            length,
            depth,
            corner_radius=corner_radius,
            cx=cx,
            cy=cy,
            base_height=base_height,
            face_selector=face_selector,
            tool=tool,
        )

    def slope_cut(
        self,
        width: float,
        length: float,
        start_depth: float,
        end_depth: float,
        *,
        cx: float = 0.0,
        cy: float = 0.0,
        slope_axis: str = "X",
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a rectangular trough with a planar sloped floor.

        Depths are measured down from the top face. ``slope_axis`` may be
        ``"X"``/``"length"`` or ``"Y"``/``"width"``.
        """
        from .operations import SlopedFloorOp

        op = SlopedFloorOp(
            cx=cx,
            cy=cy,
            width=width,
            length=length,
            start_depth=start_depth,
            end_depth=end_depth,
            slope_axis=slope_axis,
            face_selector=face_selector,
            tool=tool,
            material=self._material,
        )
        op.sequence_number = self._next_op_number
        return self._apply_op(op)

    def sloped_floor(
        self,
        width: float,
        length: float,
        start_depth: float,
        end_depth: float,
        **kwargs,
    ) -> "Stock":
        """Alias for :meth:`slope_cut`."""
        return self.slope_cut(width, length, start_depth, end_depth, **kwargs)

    def circular_pocket(
        self,
        diameter: float,
        depth: float,
        *,
        cx: float = 0.0,
        cy: float = 0.0,
        base_height: Optional[float] = None,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a circular pocket.

        Args:
            diameter: Pocket diameter (mm).
            depth: Total depth (mm).
            cx, cy: Center position on top face (mm).
            tool: ToolSpec or None.
        """
        from .operations import CircularPocketOp

        op = CircularPocketOp(
            cx=cx,
            cy=cy,
            diameter=diameter,
            depth=depth,
            base_height=base_height,
            face_selector=face_selector,
            tool=tool,
            material=self._material,
        )
        op.sequence_number = self._next_op_number
        return self._apply_op(op)

    def circular_pocket_at(
        self,
        diameter: float,
        depth: float,
        *,
        x: Optional[float] = None,
        y: Optional[float] = None,
        from_left: Optional[float] = None,
        from_right: Optional[float] = None,
        from_bottom: Optional[float] = None,
        from_top: Optional[float] = None,
        center_x: bool = False,
        center_y: bool = False,
        dx: float = 0.0,
        dy: float = 0.0,
        base_height: Optional[float] = None,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a circular pocket using natural top-face XY references."""
        cx, cy = self.resolve_xy(
            x=x,
            y=y,
            from_left=from_left,
            from_right=from_right,
            from_bottom=from_bottom,
            from_top=from_top,
            center_x=center_x,
            center_y=center_y,
            dx=dx,
            dy=dy,
        )
        return self.circular_pocket(
            diameter,
            depth,
            cx=cx,
            cy=cy,
            base_height=base_height,
            face_selector=face_selector,
            tool=tool,
        )

    def drill(
        self,
        diameter: float,
        depth: float = 0.0,
        *,
        cx: float = 0.0,
        cy: float = 0.0,
        through: bool = False,
        spot_drill: bool = True,
        peck: bool = False,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Drill a hole.

        Args:
            diameter: Hole diameter (mm).
            depth: Hole depth (mm). Ignored if through=True.
            cx, cy: Center position on top face (mm).
            through: If True, drill through entire part.
            spot_drill: If True, auto-add spot-drill first.
            peck: If True, use peck drilling cycle.
            tool: ToolSpec or None.
        """
        from .operations import DrillOp, SpotDrillOp

        result = self

        # Apply spot-drill first so it's recorded in the plan
        if spot_drill:
            spot = SpotDrillOp(
                cx=cx, cy=cy,
                hole_diameter=diameter,
                face_selector=face_selector,
                material=self._material,
            )
            spot.sequence_number = result._next_op_number
            result = result._apply_op(spot)

        # Apply drill (spot_drill=False since spot is already handled above)
        op = DrillOp(
            cx=cx,
            cy=cy,
            diameter=diameter,
            depth=depth,
            through=through,
            spot_drill=False,
            peck=peck,
            face_selector=face_selector,
            tool=tool,
            material=self._material,
        )
        op.sequence_number = result._next_op_number
        return result._apply_op(op)

    def drill_at(
        self,
        diameter: float,
        depth: float = 0.0,
        *,
        x: Optional[float] = None,
        y: Optional[float] = None,
        from_left: Optional[float] = None,
        from_right: Optional[float] = None,
        from_bottom: Optional[float] = None,
        from_top: Optional[float] = None,
        center_x: bool = False,
        center_y: bool = False,
        dx: float = 0.0,
        dy: float = 0.0,
        through: bool = False,
        spot_drill: bool = True,
        peck: bool = False,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Drill using natural top-face XY references such as edge offsets."""
        cx, cy = self.resolve_xy(
            x=x,
            y=y,
            from_left=from_left,
            from_right=from_right,
            from_bottom=from_bottom,
            from_top=from_top,
            center_x=center_x,
            center_y=center_y,
            dx=dx,
            dy=dy,
        )
        return self.drill(
            diameter,
            depth,
            cx=cx,
            cy=cy,
            through=through,
            spot_drill=spot_drill,
            peck=peck,
            face_selector=face_selector,
            tool=tool,
        )

    def corner_holes(
        self,
        diameter: float,
        depth: float = 0.0,
        *,
        inset: Optional[object] = None,
        x_inset: Optional[float] = None,
        y_inset: Optional[float] = None,
        corners: tuple[str, ...] = ("bottom_left", "bottom_right", "top_left", "top_right"),
        through: bool = False,
        spot_drill: bool = True,
        peck: bool = False,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Drill holes at named plate corners using edge insets."""
        if inset is not None:
            inset_x, inset_y = _as_pair(inset, name="inset")
            if x_inset is not None or y_inset is not None:
                raise ValueError("use either inset or x_inset/y_inset, not both")
        else:
            if x_inset is None or y_inset is None:
                raise ValueError("corner_holes requires inset or both x_inset and y_inset")
            inset_x, inset_y = float(x_inset), float(y_inset)

        corner_refs = {
            "bottom_left": dict(from_left=inset_x, from_bottom=inset_y),
            "bottom_right": dict(from_right=inset_x, from_bottom=inset_y),
            "top_left": dict(from_left=inset_x, from_top=inset_y),
            "top_right": dict(from_right=inset_x, from_top=inset_y),
        }
        result = self
        for corner in corners:
            key = str(corner).lower().replace("-", "_")
            if key not in corner_refs:
                raise ValueError(f"unknown corner {corner!r}")
            cx, cy = result.resolve_xy(**corner_refs[key])
            result = result.drill(
                diameter,
                depth,
                cx=cx,
                cy=cy,
                through=through,
                spot_drill=spot_drill,
                peck=peck,
                face_selector=face_selector,
                tool=tool,
            )
        return result

    def drill_symmetric(
        self,
        diameter: float,
        depth: float = 0.0,
        *,
        mirror_x: bool = False,
        mirror_y: bool = False,
        x: Optional[float] = None,
        y: Optional[float] = None,
        from_left: Optional[float] = None,
        from_right: Optional[float] = None,
        from_bottom: Optional[float] = None,
        from_top: Optional[float] = None,
        center_x: bool = False,
        center_y: bool = False,
        dx: float = 0.0,
        dy: float = 0.0,
        through: bool = False,
        spot_drill: bool = True,
        peck: bool = False,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Drill one hole and its mirrors across the stock centerlines."""
        if not mirror_x and not mirror_y:
            raise ValueError("drill_symmetric requires mirror_x, mirror_y, or both")
        point = self.resolve_xy(
            x=x,
            y=y,
            from_left=from_left,
            from_right=from_right,
            from_bottom=from_bottom,
            from_top=from_top,
            center_x=center_x,
            center_y=center_y,
            dx=dx,
            dy=dy,
        )
        result = self
        for cx, cy in self._mirror_xy_points([point], mirror_x=mirror_x, mirror_y=mirror_y):
            result = result.drill(
                diameter,
                depth,
                cx=cx,
                cy=cy,
                through=through,
                spot_drill=spot_drill,
                peck=peck,
                face_selector=face_selector,
                tool=tool,
            )
        return result

    def drill_pattern(
        self,
        diameter: float,
        depth: float = 0.0,
        *,
        rows: int,
        cols: int,
        spacing_x: Optional[float] = None,
        spacing_y: Optional[float] = None,
        spacing: Optional[object] = None,
        center_x: float = 0.0,
        center_y: float = 0.0,
        through: bool = False,
        spot_drill: bool = True,
        peck: bool = False,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Drill a centered rectangular row/column pattern."""
        if rows <= 0 or cols <= 0:
            raise ValueError("rows and cols must be positive")
        if spacing is not None:
            sx, sy = _as_pair(spacing, name="spacing")
            if spacing_x is not None or spacing_y is not None:
                raise ValueError("use either spacing or spacing_x/spacing_y, not both")
        else:
            if spacing_x is None or spacing_y is None:
                raise ValueError("drill_pattern requires spacing or both spacing_x and spacing_y")
            sx, sy = float(spacing_x), float(spacing_y)

        result = self
        x0 = float(center_x) - (cols - 1) * sx / 2.0
        y0 = float(center_y) - (rows - 1) * sy / 2.0
        for row in range(rows):
            for col in range(cols):
                result = result.drill(
                    diameter,
                    depth,
                    cx=x0 + col * sx,
                    cy=y0 + row * sy,
                    through=through,
                    spot_drill=spot_drill,
                    peck=peck,
                    face_selector=face_selector,
                    tool=tool,
                )
        return result

    def tap(
        self,
        diameter: float,
        thread_pitch: float = 0.0,
        *,
        depth: float = 10.0,
        cx: float = 0.0,
        cy: float = 0.0,
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Tap a pre-drilled hole.

        Geometry is unchanged (thread form too fine for B-Rep).

        Args:
            diameter: Nominal thread diameter (mm), e.g. 6 for M6.
            thread_pitch: Thread pitch (mm). Auto-derived if 0.
            depth: Tap depth (mm).
            cx, cy: Center position (mm).
            tool: ToolSpec or None.
        """
        from .operations import TapOp

        op = TapOp(
            cx=cx,
            cy=cy,
            diameter=diameter,
            pitch=thread_pitch,
            depth=depth,
            material=self._material,
            tool=tool,
        )
        op.sequence_number = self._next_op_number
        return self._apply_op(op)

    def threaded_hole(
        self,
        diameter: float,
        depth: float = 0.0,
        *,
        cx: float = 0.0,
        cy: float = 0.0,
        through: bool = False,
    ) -> "Stock":
        """Convenience: spot-drill + drill + tap in one call.

        Arg:
            diameter: Nominal thread diameter (mm), e.g. 8 for M8.
            depth: Hole / tap depth (mm). 0 = through.
            cx, cy: Center position (mm).
            through: If True, drill through entire part.
        """
        from .operations import threaded_hole_ops

        ops = threaded_hole_ops(
            diameter=diameter,
            depth=depth,
            position=(cx, cy),
            material=self._material,
            through=through,
        )
        result = self
        for op in ops:
            op.sequence_number = result._next_op_number
            result = result._apply_op(op)
        return result

    def contour(
        self,
        depth: float,
        *,
        tool: Optional[ToolSpec] = None,
        stepdown: Optional[float] = None,
    ) -> "Stock":
        """Profile / contour milling around the outer boundary.

        Args:
            depth: Total depth to profile (mm).
            tool: ToolSpec or None.
            stepdown: Depth per Z-level pass (default 0.3 * tool_dia).
        """
        from .operations import ContourOp

        op = ContourOp(
            depth=depth,
            stepdown=stepdown or 1.0,
            tool=tool,
            material=self._material,
        )
        op.sequence_number = self._next_op_number
        return self._apply_op(op)

    def slot(
        self,
        length: float,
        width: float,
        depth: float = 5.0,
        *,
        angle: float = 0.0,
        cx: float = 0.0,
        cy: float = 0.0,
        through: bool = False,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a straight slot (obround).

        Args:
            length: Total slot length (mm).
            width: Slot width (mm).
            depth: Slot depth (mm).
            angle: Rotation angle from X axis (degrees).
            cx, cy: Center position (mm).
            through: If True, cut through entire part.
            tool: ToolSpec or None.
        """
        from .operations import SlotOp

        op = SlotOp(
            cx=cx,
            cy=cy,
            length=length,
            width=width,
            depth=depth,
            angle=angle,
            through=through,
            face_selector=face_selector,
            tool=tool,
            material=self._material,
        )
        op.sequence_number = self._next_op_number
        return self._apply_op(op)

    def slot_at(
        self,
        length: float,
        width: float,
        depth: float = 5.0,
        *,
        angle: float = 0.0,
        x: Optional[float] = None,
        y: Optional[float] = None,
        from_left: Optional[float] = None,
        from_right: Optional[float] = None,
        from_bottom: Optional[float] = None,
        from_top: Optional[float] = None,
        center_x: bool = False,
        center_y: bool = False,
        dx: float = 0.0,
        dy: float = 0.0,
        through: bool = False,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a slot using natural top-face XY references."""
        cx, cy = self.resolve_xy(
            x=x,
            y=y,
            from_left=from_left,
            from_right=from_right,
            from_bottom=from_bottom,
            from_top=from_top,
            center_x=center_x,
            center_y=center_y,
            dx=dx,
            dy=dy,
        )
        return self.slot(
            length,
            width,
            depth,
            angle=angle,
            cx=cx,
            cy=cy,
            through=through,
            face_selector=face_selector,
            tool=tool,
        )

    def chamfer(
        self,
        width: float = 0.5,
        *,
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Chamfer the top edges.

        Args:
            width: Chamfer width (mm).
            tool: ToolSpec or None.
        """
        from .operations import ChamferOp

        op = ChamferOp(
            width=width,
            tool=tool,
            material=self._material,
        )
        op.sequence_number = self._next_op_number
        return self._apply_op(op)

    def peck_drill(
        self,
        diameter: float,
        depth: float = 20.0,
        *,
        cx: float = 0.0,
        cy: float = 0.0,
        peck_step: float = 2.0,
        through: bool = False,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Peck-drill a deep hole."""
        from .operations import PeckDrillOp

        op = PeckDrillOp(
            cx=cx,
            cy=cy,
            diameter=diameter,
            depth=depth,
            peck_step=peck_step,
            through=through,
            face_selector=face_selector,
            tool=tool,
            material=self._material,
        )
        return self._apply_op(op)

    def ream(
        self,
        diameter: float,
        depth: float = 10.0,
        *,
        cx: float = 0.0,
        cy: float = 0.0,
        through: bool = False,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Finish a pre-drilled hole with a reamer."""
        from .operations import ReamOp

        op = ReamOp(
            cx=cx,
            cy=cy,
            diameter=diameter,
            depth=depth,
            through=through,
            tool=tool,
            material=self._material,
        )
        return self._apply_op(op)

    def bore(
        self,
        diameter: float,
        depth: float = 10.0,
        *,
        cx: float = 0.0,
        cy: float = 0.0,
        through: bool = False,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Bore a hole for trueness and finish."""
        from .operations import BoreOp

        op = BoreOp(
            cx=cx,
            cy=cy,
            diameter=diameter,
            depth=depth,
            through=through,
            tool=tool,
            material=self._material,
        )
        return self._apply_op(op)

    def countersink(
        self,
        diameter: float,
        countersink_diameter: float,
        *,
        depth: float = 0.0,
        countersink_angle: float = 82.0,
        cx: float = 0.0,
        cy: float = 0.0,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a countersunk hole."""
        from .operations import CountersinkOp

        op = CountersinkOp(
            cx=cx,
            cy=cy,
            diameter=diameter,
            countersink_diameter=countersink_diameter,
            depth=depth,
            countersink_angle=countersink_angle,
            face_selector=face_selector,
            tool=tool,
            material=self._material,
        )
        return self._apply_op(op)

    def counterbore(
        self,
        hole_diameter: float,
        counterbore_diameter: float,
        counterbore_depth: float,
        *,
        cx: float = 0.0,
        cy: float = 0.0,
        through: bool = False,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a flat-bottom counterbore."""
        from .operations import CounterboreOp

        op = CounterboreOp(
            cx=cx,
            cy=cy,
            hole_diameter=hole_diameter,
            counterbore_diameter=counterbore_diameter,
            counterbore_depth=counterbore_depth,
            through=through,
            face_selector=face_selector,
            tool=tool,
            material=self._material,
        )
        return self._apply_op(op)

    def thread_mill(
        self,
        diameter: float,
        depth: float = 10.0,
        *,
        pitch: float = 0.0,
        cx: float = 0.0,
        cy: float = 0.0,
        through: bool = False,
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Thread-mill an internal thread."""
        from .operations import ThreadMillOp

        op = ThreadMillOp(
            cx=cx,
            cy=cy,
            diameter=diameter,
            pitch=pitch,
            depth=depth,
            through=through,
            tool=tool,
            material=self._material,
        )
        return self._apply_op(op)

    def t_slot(
        self,
        length: float,
        width: float,
        depth: float = 8.0,
        *,
        angle: float = 0.0,
        cx: float = 0.0,
        cy: float = 0.0,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a T-slot, approximated as a rectangular slot."""
        from .operations import TSlotOp

        op = TSlotOp(
            cx=cx,
            cy=cy,
            length=length,
            width=width,
            depth=depth,
            angle=angle,
            face_selector=face_selector,
            tool=tool,
            material=self._material,
        )
        return self._apply_op(op)

    def dovetail(
        self,
        length: float,
        width: float,
        depth: float = 5.0,
        *,
        angle: float = 45.0,
        cx: float = 0.0,
        cy: float = 0.0,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a dovetail slot."""
        from .operations import DovetailOp

        op = DovetailOp(
            cx=cx,
            cy=cy,
            length=length,
            width=width,
            depth=depth,
            angle=angle,
            face_selector=face_selector,
            tool=tool,
            material=self._material,
        )
        return self._apply_op(op)

    def groove(
        self,
        length: float,
        width: float,
        depth: float = 5.0,
        *,
        angle: float = 0.0,
        cx: float = 0.0,
        cy: float = 0.0,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a rectangular groove."""
        from .operations import GrooveOp

        op = GrooveOp(
            cx=cx,
            cy=cy,
            length=length,
            width=width,
            depth=depth,
            angle=angle,
            tool=tool,
            material=self._material,
        )
        return self._apply_op(op)

    def surface_3d(
        self,
        *,
        stepover: float = 1.0,
        depth: float = 0.5,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Record a 3D surfacing pass."""
        from .operations import Surface3DOp

        op = Surface3DOp(
            stepover=stepover,
            depth=depth,
            face_selector=face_selector,
            tool=tool,
            material=self._material,
        )
        return self._apply_op(op)

    def deburr(
        self,
        *,
        edge_selection: str = "all",
        tool_diameter: float = 3.0,
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Deburr edges using a small chamfer proxy."""
        from .operations import DeburrOp

        op = DeburrOp(
            edge_selection=edge_selection,
            tool_diameter=tool_diameter,
            tool=tool,
            material=self._material,
        )
        return self._apply_op(op)

    def spot_face(
        self,
        diameter: float,
        depth: float = 0.5,
        *,
        cx: float = 0.0,
        cy: float = 0.0,
        face_selector: str = ">Z",
        tool: Optional[ToolSpec] = None,
    ) -> "Stock":
        """Cut a shallow flat spot-face."""
        from .operations import SpotFaceOp

        op = SpotFaceOp(
            cx=cx,
            cy=cy,
            diameter=diameter,
            depth=depth,
            face_selector=face_selector,
            tool=tool,
            material=self._material,
        )
        return self._apply_op(op)

    def edge_chamfer(self, selector, width: float, *, angle: float = 45.0,
                     tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import EdgeChamferOp
        return self._apply_op(EdgeChamferOp(
            selector=selector, width=width, angle=angle,
            tool=tool, material=self._material,
        ))

    def edge_fillet(self, selector, radius: float, *,
                    tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import EdgeFilletOp
        return self._apply_op(EdgeFilletOp(
            selector=selector, radius=radius, tool=tool,
            material=self._material,
        ))

    def deburr_edges(self, selector="all", width: float = 0.2, *,
                     tool: Optional[ToolSpec] = None) -> "Stock":
        return self.edge_chamfer(selector, width, angle=45.0, tool=tool)

    def profile_pocket(self, profile, depth: float, *, face_selector: str = ">Z",
                       through: bool = False, islands=None,
                       stepdown: float = 1.0, finish_allowance: float = 0.1, chord_error: float = 0.01,
                       cx: Optional[float] = None,
                       cy: Optional[float] = None,
                       tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import ProfilePocketOp
        profile = _place_profile(profile, cx, cy)
        width, length = _profile_bounds_for_stock(profile)
        if through:
            depth = _profile_through_depth(self, face_selector)
        return self._apply_op(ProfilePocketOp(
            profile=profile, islands=islands, width=width, length=length,
            depth=depth, through=through, face_selector=face_selector,
            tool=tool, material=self._material, stepdown=stepdown, finish_allowance=finish_allowance, chord_error=chord_error,
        ))

    def profile_cutout(self, profile, *, depth: Optional[float] = None,
                       through: bool = False, face_selector: str = ">Z",
                       stepdown: float = 1.0, finish_allowance: float = 0.1, chord_error: float = 0.01,
                       cx: Optional[float] = None,
                       cy: Optional[float] = None,
                       tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import ProfileCutoutOp
        profile = _place_profile(profile, cx, cy)
        width, length = _profile_bounds_for_stock(profile)
        cut_depth = depth if depth is not None else self._stock_dims.get("height", 1.0)
        if through:
            cut_depth = _profile_through_depth(self, face_selector)
        return self._apply_op(ProfileCutoutOp(
            profile=profile, width=width, length=length, depth=cut_depth,
            through=through, face_selector=face_selector,
            tool=tool, material=self._material, stepdown=stepdown, finish_allowance=finish_allowance, chord_error=chord_error,
        ))

    def laser_cut_profile(self, profile, *, kerf_width: float = 0.15,
                          assist_gas: str = "nitrogen",
                          cut_speed_mm_min: float = 2400.0,
                          pierce_time_s: float = 0.15,
                          face_selector: str = ">Z",
                          cx: Optional[float] = None,
                          cy: Optional[float] = None) -> "Stock":
        """Laser-cut the outside sheet blank and retain the supplied profile."""
        from .operations import LaserCutProfileOp
        profile = _place_profile(profile, cx, cy)
        return self._apply_op(LaserCutProfileOp(
            profile=profile,
            depth=float(self._stock_dims.get("thickness", self._stock_dims.get("height", 1.0))),
            kerf_width=kerf_width,
            assist_gas=assist_gas,
            cut_speed_mm_min=cut_speed_mm_min,
            pierce_time_s=pierce_time_s,
            face_selector=face_selector,
            material=self._material,
        ))

    def laser_cut_internal(self, profile, *, kerf_width: float = 0.15,
                           assist_gas: str = "nitrogen",
                           cut_speed_mm_min: float = 2400.0,
                           pierce_time_s: float = 0.15,
                           face_selector: str = ">Z",
                           cx: Optional[float] = None,
                           cy: Optional[float] = None) -> "Stock":
        """Laser-cut an internal through-profile in the sheet blank."""
        from .operations import LaserCutInternalOp
        profile = _place_profile(profile, cx, cy)
        return self._apply_op(LaserCutInternalOp(
            profile=profile,
            depth=float(self._stock_dims.get("thickness", self._stock_dims.get("height", 1.0))),
            kerf_width=kerf_width,
            assist_gas=assist_gas,
            cut_speed_mm_min=cut_speed_mm_min,
            pierce_time_s=pierce_time_s,
            face_selector=face_selector,
            material=self._material,
        ))

    def laser_cut_hole(self, diameter: float, *, cx: float = 0.0, cy: float = 0.0,
                       **kwargs) -> "Stock":
        """Laser-cut a round through-hole in sheet stock."""
        profile = {"type": "circle", "diameter": diameter, "cx": cx, "cy": cy}
        return self.laser_cut_internal(profile, **kwargs)

    def laser_cut_slot(self, length: float, width: float, *, cx: float = 0.0,
                       cy: float = 0.0, angle: float = 0.0,
                       **kwargs) -> "Stock":
        """Laser-cut a rounded slot through sheet stock."""
        profile = {
            "type": "slot",
            "length": length,
            "width": width,
            "cx": cx,
            "cy": cy,
            "angle": angle,
        }
        return self.laser_cut_internal(profile, **kwargs)

    def press_brake_bend(self, *, axis: str = "X", position: float = 0.0,
                         angle: float = 90.0, side: str = "positive",
                         radius: float = 1.0,
                         axis_z: Optional[float] = None,
                         tooling: str = "air_bend_v_die") -> "Stock":
        """Form one side of the sheet around a straight press-brake bend line."""
        from .operations import PressBrakeBendOp
        thickness = float(self._stock_dims.get("thickness", self._stock_dims.get("height", 1.0)))
        if axis_z is None:
            axis_z = thickness / 2.0
        op = PressBrakeBendOp(
            axis=axis,
            bend_position=position,
            angle=angle,
            side=side,
            radius=radius,
            sheet_thickness=thickness,
            axis_z=axis_z,
            tooling=tooling,
            material=self._material,
        )
        return self._apply_op(op)

    def bend(self, **kwargs) -> "Stock":
        """Alias for :meth:`press_brake_bend`."""
        return self.press_brake_bend(**kwargs)

    def profile_contour(self, profile, depth: float, *, side: str = "outside",
                        face_selector: str = ">Z",
                        stepdown: float = 1.0,
                        finish_allowance: float = 0.1,
                        chord_error: float = 0.01,
                        cx: Optional[float] = None,
                        cy: Optional[float] = None,
                        tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import ProfileContourOp
        profile = _place_profile(profile, cx, cy)
        return self._apply_op(ProfileContourOp(
            profile=profile, depth=depth, side=side, face_selector=face_selector,
            tool=tool, material=self._material, stepdown=stepdown,
            finish_allowance=finish_allowance, chord_error=chord_error,
        ))

    def machine_around_profile(self, profile, height: float, *,
                               stock_envelope=None,
                               base_height: Optional[float] = None,
                               cx: Optional[float] = None,
                               cy: Optional[float] = None,
                               face_selector: str = ">Z",
                               tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import MachineAroundProfileOp
        if isinstance(profile, dict) and (cx is not None or cy is not None):
            profile = dict(profile)
            if cx is not None:
                profile["cx"] = cx
            if cy is not None:
                profile["cy"] = cy
        width, length = _profile_bounds_for_stock(profile)
        return self._apply_op(MachineAroundProfileOp(
            profile=profile, width=width, length=length, depth=height,
            height=height, stock_envelope=stock_envelope,
            base_height=base_height,
            face_selector=face_selector,
            tool=tool, material=self._material,
        ))

    def machine_around_profiles(self, profiles, height: float, *,
                                stock_envelope=None,
                                base_height: Optional[float] = None,
                                face_selector: str = ">Z",
                                tool: Optional[ToolSpec] = None) -> "Stock":
        """Machine around multiple retained bosses/ribs/pads in one cut."""
        from .operations import MachineAroundProfilesOp
        profiles = list(profiles or [])
        width, length = _profiles_bounds_for_stock(profiles)
        return self._apply_op(MachineAroundProfilesOp(
            profiles=profiles, profile=profiles, width=width, length=length,
            depth=height, height=height, stock_envelope=stock_envelope,
            base_height=base_height,
            face_selector=face_selector, tool=tool, material=self._material,
        ))

    def machine_around_cylinder(self, diameter: float, height: float, *,
                                cx: float = 0.0, cy: float = 0.0,
                                base_height: Optional[float] = None,
                                face_selector: str = ">Z",
                                tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import MachineAroundCylinderOp
        return self._apply_op(MachineAroundCylinderOp(
            cx=cx, cy=cy, diameter=diameter, depth=height, height=height,
            base_height=base_height, face_selector=face_selector,
            tool=tool, material=self._material,
        ))

    def rib(self, width: float, length: float, height: float, *, cx: float = 0.0,
            cy: float = 0.0, angle: float = 0.0,
            face_selector: str = ">Z",
            tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import RibOp
        profile = {
            "type": "rib",
            "width": width,
            "length": length,
            "cx": cx,
            "cy": cy,
            "angle": angle,
        }
        return self._apply_op(RibOp(
            profile=profile, width=width, length=length, depth=height,
            height=height, angle=angle, face_selector=face_selector,
            tool=tool, material=self._material,
        ))

    def pad(self, profile, height: float, *, tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import PadOp
        width, length = _profile_bounds_for_stock(profile)
        return self._apply_op(PadOp(
            profile=profile, width=width, length=length, depth=height,
            height=height, tool=tool, material=self._material,
        ))

    def _turning_frame(self, axis=None, origin=None) -> dict:
        return {"axis": axis or self._stock_dims.get("axis", "Z"),
            "origin": origin if origin is not None else self._stock_dims.get("spindle_origin_mm", (0, 0, 0))}

    def turn_profile(self, profile, *, axis: Optional[str] = None, origin=None,
                     stock_diameter: Optional[float] = None,
                     tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import TurnProfileOp
        return self._apply_op(TurnProfileOp(
            profile=profile, **self._turning_frame(axis, origin), stock_diameter=stock_diameter,
            tool=tool, material=self._material,
        ))

    def turn_face(self, z: float, *, axis: Optional[str] = None, origin=None, side: str = "front",
                  tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import TurnFaceOp
        return self._apply_op(TurnFaceOp(z=z, **self._turning_frame(axis, origin), side=side, tool=tool, material=self._material))

    def turn_od(self, diameter: float, length: float, *,
                axis: Optional[str] = None, origin=None, start_z: Optional[float] = None,
                tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import TurnODOp
        return self._apply_op(TurnODOp(
            diameter=diameter, length=length, tool=tool, material=self._material,
            **self._turning_frame(axis, origin), start_z=start_z,
        ))

    def turn_id(self, diameter: float, depth: float, *,
                axis: Optional[str] = None, origin=None, start_z: Optional[float] = None,
                tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import TurnIDOp
        return self._apply_op(TurnIDOp(
            diameter=diameter, length=depth, depth=depth,
            tool=tool, material=self._material,
            **self._turning_frame(axis, origin), start_z=start_z,
        ))

    def turn_groove(self, width: float, depth: float, z: float, *,
                    axis: Optional[str] = None, origin=None, diameter: Optional[float] = None, internal: bool = False,
                    tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import TurnGrooveOp
        return self._apply_op(TurnGrooveOp(
            width=width, depth=depth, z=z, tool=tool, material=self._material,
            **self._turning_frame(axis, origin), diameter=diameter, internal=internal,
        ))

    def turn_thread(self, diameter: float, pitch: float, length: float, *,
                    internal: bool = False,
                    axis: Optional[str] = None, origin=None, start_z: Optional[float] = None,
                    thread_depth: Optional[float] = None, angle: float = 60.0, left_handed: bool = False,
                    tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import TurnThreadOp
        return self._apply_op(TurnThreadOp(
            diameter=diameter, pitch=pitch, length=length, internal=internal,
            tool=tool, material=self._material,
            **self._turning_frame(axis, origin), start_z=start_z,
            thread_depth=thread_depth, angle=angle, left_handed=left_handed,
        ))

    def mill_turn_setup(self, axis: str = "Z", work_offset: str = "G54") -> "Stock":
        from .operations import MillTurnSetupOp
        return self._apply_op(MillTurnSetupOp(
            axis=axis, work_offset=work_offset, material=self._material,
        ))

    def thin_wall_pocket(self, profile, wall_thickness: float, depth: float, *,
                         open_faces=None,
                         tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import ThinWallPocketOp
        return self._apply_op(ThinWallPocketOp(
            profile=profile, wall_thickness=wall_thickness, depth=depth,
            open_faces=open_faces, tool=tool, material=self._material,
        ))

    def thin_wall_shell(self, wall_thickness: float, *,
                        tool: Optional[ToolSpec] = None) -> "Stock":
        """Offset-shell the current machined shape by wall thickness."""
        from .operations import ThinWallShellOp
        return self._apply_op(ThinWallShellOp(
            wall_thickness=wall_thickness, tool=tool, material=self._material,
        ))

    def shell_wall(self, wall_thickness: float, **kwargs) -> "Stock":
        """Alias for :meth:`thin_wall_shell`."""
        return self.thin_wall_shell(wall_thickness, **kwargs)

    def hollow_bore(self, inner_profile, outer_profile, depth: float, *,
                    access_face: str = ">Z",
                    tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import HollowBoreOp
        return self._apply_op(HollowBoreOp(
            inner_profile=inner_profile, outer_profile=outer_profile,
            depth=depth, access_face=access_face, tool=tool,
            material=self._material,
        ))

    def tube_profile(self, outer_profile, inner_profile, length: float, *,
                     process: str = "mill_turn",
                     axis: str = "X",
                     cx: float = 0.0,
                     cy: float = 0.0,
                     cz: float = 0.0,
                     start: Optional[float] = None,
                     end: Optional[float] = None,
                     combine: str = "union",
                     tool: Optional[ToolSpec] = None) -> "Stock":
        """Represent a straight hollow tube retained from stock.

        `axis` may be X, Y, or Z. For horizontal tube bosses, pass start/end
        along that axis plus the other two center coordinates in `cx/cy/cz` or
        in the profile dict. The default `combine="union"` keeps this usable in
        operation chains that first define a base pad and then add retained tube
        intent; `combine="replace"` can be used for standalone tube parts.
        """
        from .operations import TubeProfileOp
        return self._apply_op(TubeProfileOp(
            outer_profile=outer_profile, inner_profile=inner_profile,
            length=length, process=process, axis=axis, cx=cx, cy=cy, cz=cz,
            start=start, end=end, combine=combine,
            tool=tool, material=self._material,
        ))

    def surface_mill(self, surface_ref, *, tolerance_mm: float = 0.10,
                     tool: Optional[ToolSpec] = None,
                     strategy: str = "parallel") -> "Stock":
        from .operations import SurfaceMillOp
        return self._apply_op(SurfaceMillOp(
            surface_ref=surface_ref, tolerance_mm=tolerance_mm,
            strategy=strategy, tool=tool, material=self._material,
        ))

    def sweep_mill(self, profile, path, *, tolerance_mm: float = 0.10,
                   tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import SweepMillOp
        return self._apply_op(SweepMillOp(
            profile=profile, path=path, tolerance_mm=tolerance_mm,
            tool=tool, material=self._material,
        ))

    def loft_mill(self, profiles, *, tolerance_mm: float = 0.10,
                  tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import LoftMillOp
        return self._apply_op(LoftMillOp(
            profiles=profiles, tolerance_mm=tolerance_mm,
            tool=tool, material=self._material,
        ))

    def dome_mill(self, radius: float, height: float, *, cx: float = 0.0,
                  cy: float = 0.0, tolerance_mm: float = 0.10,
                  tool: Optional[ToolSpec] = None) -> "Stock":
        from .operations import DomeMillOp
        return self._apply_op(DomeMillOp(
            radius=radius, height=height, cx=cx, cy=cy,
            tolerance_mm=tolerance_mm, tool=tool, material=self._material,
        ))

    def wire_cut_profile(self, profile, thickness: float, *, taper: float = 0.0,
                         tolerance_mm: float = 0.05) -> "Stock":
        from .operations import WireCutProfileOp
        return self._apply_op(WireCutProfileOp(
            profile=profile, thickness=thickness, taper=taper,
            tolerance_mm=tolerance_mm, material=self._material,
        ))

    def wire_cut_internal(self, profile, thickness: float, *, start_hole=None,
                          tolerance_mm: float = 0.05) -> "Stock":
        from .operations import WireCutInternalOp
        return self._apply_op(WireCutInternalOp(
            profile=profile, thickness=thickness, start_hole=start_hole,
            tolerance_mm=tolerance_mm, material=self._material,
        ))

    # -------------------------------------------------------------------
    #  Fixturing API
    # -------------------------------------------------------------------

    def with_fixture(self, fixture_spec: "FixtureSpec") -> "Stock":
        """Attach a FixtureSpec to this Stock. Returns new Stock."""
        return Stock._from_parts(
            self._shape,
            self._material,
            self._stock_dims,
            self._plan,
            self._next_op_number,
            fixture=fixture_spec,
            setups=dict(self._setups),
            active_setup=self._active_setup,
            state_history=list(self._state_history),
        )

    def new_setup(self, name: str, face_selector: str = ">Z",
                  work_offset: str = "G54",
                  work_offset_values=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                  description: str = "",
                  rotation_deg: float = 0.0) -> "Stock":
        """Define a new machining setup. Requires .with_fixture() to have been called first."""
        from .fixturing import Setup

        if self._fixture is None:
            raise ValueError("No fixture assigned. Call .with_fixture() first.")
        setup = Setup(name=name, fixture=self._fixture, face_selector=face_selector,
                      work_offset=work_offset, work_offset_values=work_offset_values,
                      rotation_deg=rotation_deg, description=description)
        new_setups = dict(self._setups)
        new_setups[name] = setup
        return Stock._from_parts(
            self._shape,
            self._material,
            self._stock_dims,
            self._plan,
            self._next_op_number,
            fixture=self._fixture,
            setups=new_setups,
            active_setup=name,
            state_history=list(self._state_history),
        )

    def use_setup(self, name: str) -> "Stock":
        """Switch to an existing setup by name."""
        if name not in self._setups:
            raise KeyError(f"Setup {name!r} not found.")
        return Stock._from_parts(
            self._shape,
            self._material,
            self._stock_dims,
            self._plan,
            self._next_op_number,
            fixture=self._fixture,
            setups=dict(self._setups),
            active_setup=name,
            state_history=list(self._state_history),
        )

    def validate_fixture_clearance(self) -> list[str]:
        """Check all operations against the active fixture's clamping zones."""
        from .fixturing import check_operation_against_fixture

        warnings = []
        active_name = self._active_setup
        if active_name is None or active_name not in self._setups:
            return warnings
        active_setup = self._setups[active_name]
        for op_dict in self._plan.operations:
            warnings.extend(
                check_operation_against_fixture(op_dict, active_setup, self._stock_dims)
            )
        return warnings

    # -------------------------------------------------------------------
    #  Simulation bridge
    # -------------------------------------------------------------------

    def simulate(self, toolpath_precision: float = 0.5) -> dict:
        """Simulate material removal via the tri-dexel engine.

        Returns a dict with:
          - final_volume_mm3
          - volume_removed_mm3
          - gouges: list of gouge descriptors
          - cycle_time_minutes
          - ops_simulated: count of operations that ran through the engine
        """
        try:
            from src.simulation.material_removal import MaterialRemovalEngine
            from src.simulation.tools import create_tool as sim_create_tool
            from src.simulation.tri_dexel import TriDexelModel
            import numpy as np
        except ImportError:
            # Fallback: try without the src. prefix (if running from within src/)
            try:
                from simulation.material_removal import MaterialRemovalEngine
                from simulation.tools import create_tool as sim_create_tool
                from simulation.tri_dexel import TriDexelModel
                import numpy as np
            except ImportError as e:
                return {"error": f"Simulation not available: {e}"}

        # 1. Build initial tri-dexel grid from the original stock mesh.
        # Process-video and validation replay must not start from the already
        # finished CadQuery shape.
        try:
            from .operation_video import _initial_stock_mesh, _mesh_bbox

            mesh, _initial_source = _initial_stock_mesh(self)
            bbox = _mesh_bbox(mesh) if mesh is not None else self.bounding_box
        except Exception:
            mesh = self.to_mesh()
            bbox = self.bounding_box
        if mesh is None:
            return {"error": "Cannot create mesh from shape."}

        dexel_resolution = float(toolpath_precision)  # mm per dexel

        # Create TriDexelModel from the mesh, then the MaterialRemovalEngine
        tri_model = TriDexelModel.from_mesh(mesh, resolution=dexel_resolution)
        engine = MaterialRemovalEngine(
            tri_dexel=tri_model,
            linear_resolution=dexel_resolution,
        )
        initial_volume = engine.tri_dexel.volume()

        # Tool-type mapping: SubCAD tool_type -> simulation create_tool key
        _TOOL_TYPE_MAP = {
            "flat_endmill": "flat_endmill",
            "ball_endmill": "ball_endmill",
            "drill": "drill",
            "chamfer": "chamfer",
            "thread_mill": "thread_mill",
            "dovetail": "dovetail",
            "end_mill": "flat_endmill",       # alias
            "face_mill": "flat_endmill",      # alias
            "spot_drill": "drill",            # alias
        }

        # 2. Replay each operation's toolpath
        gouges = []
        operation_results = []
        ops_simulated = 0
        for op_dict in self._plan.operations:
            op_result = {
                "sequence_number": op_dict.get("sequence_number"),
                "operation": op_dict.get("operation", "unknown"),
                "tool_id": (op_dict.get("tool") or {}).get("catalog_id", ""),
                "estimated_time_minutes": (
                    (op_dict.get("toolpath_summary") or {}).get("estimated_time_min")
                    or (op_dict.get("toolpath_summary") or {}).get("estimated_time_minutes")
                    or (op_dict.get("pass_plan") or {}).get("estimated_time_min")
                ),
                "status": "pending",
            }
            op_tool_type = op_dict.get("tool_type", "flat_endmill")
            op_tool_dia = op_dict.get("tool_diameter_mm", 10.0)

            # Map SubCAD tool type to simulation tool type
            sim_tool_key = _TOOL_TYPE_MAP.get(op_tool_type, None)
            if sim_tool_key is None:
                # Unknown tool type - skip with a warning-level note
                gouges.append({
                    "operation": op_dict.get("operation", "unknown"),
                    "message": f"Skipped: unknown tool_type={op_tool_type!r}",
                })
                op_result["status"] = "skipped"
                op_result["message"] = f"unknown tool_type={op_tool_type!r}"
                operation_results.append(op_result)
                continue

            try:
                sim_tool = sim_create_tool(sim_tool_key, diameter=float(op_tool_dia))
            except (TypeError, ValueError) as e:
                gouges.append({
                    "operation": op_dict.get("operation", "unknown"),
                    "message": f"Tool creation failed: {e}",
                })
                op_result["status"] = "error"
                op_result["message"] = f"Tool creation failed: {e}"
                operation_results.append(op_result)
                continue

            # Prefer operation-provided toolpath data when available.
            toolpath_data = op_dict.get("toolpath")
            toolpath = (
                _toolpath_from_serialized(
                    toolpath_data,
                    z_offset=float(bbox.get("z_max", 0.0) or 0.0),
                )
                if toolpath_data
                else _op_to_simple_toolpath(op_dict, bbox)
            )
            if not toolpath:
                toolpath = _op_to_simple_toolpath(op_dict, bbox)
            for pose in toolpath:
                engine.remove_tool_at_pose(sim_tool, pose)

            ops_simulated += 1
            op_result["status"] = "simulated"
            operation_results.append(op_result)

            # Check for gouges (material removed below target)
            if engine.tri_dexel.volume() < 0:
                gouges.append({
                    "operation": op_dict.get("operation", "unknown"),
                    "message": "Gouge detected -- volume went below zero",
                })

        final_volume = max(0.0, engine.tri_dexel.volume())
        if any(op.get("status") == "error" for op in operation_results):
            status = "error"
        elif gouges:
            status = "warning"
        else:
            status = "pass"
        return {
            "status": status,
            "final_volume_mm3": round(final_volume, 1),
            "volume_removed_mm3": round(initial_volume - final_volume, 1),
            "gouges": gouges,
            "cycle_time_minutes": self._plan.estimated_time_minutes,
            "operations": operation_results,
            "per_operation_timing": operation_results,
            "ops_simulated": ops_simulated,
        }

    # -------------------------------------------------------------------
    #  Output
    # -------------------------------------------------------------------

    def to_step(self, path: str) -> None:
        """Export the current shape to a STEP file."""
        export_step(self._shape, path)

    def to_stl(self, path: str) -> None:
        """Export the current shape to an STL file."""
        export_stl(self._shape, path)

    def to_mesh(self):
        """Convert the current shape to a trimesh.Trimesh."""
        return to_mesh(self._shape)

    def process_plan(self) -> dict:
        """Return the accumulated process plan as a serializable dict."""
        return self._plan.to_dict()

    def save_process_plan(self, path: str) -> None:
        """Save the process plan as a JSON file."""
        self._plan.save(path)

    def setup_sheet(self, economics=None) -> dict:
        """Return a shop-floor setup sheet dictionary."""
        return self._plan.setup_sheet_dict(economics=economics)

    def estimate_cost(
        self,
        machine: str = "generic_3axis_mill",
        quantity: int = 1,
        currency: Optional[str] = None,
    ) -> dict:
        """Return an engineering time/cost estimate for this stock/process plan."""
        from .economics import estimate_cost

        return estimate_cost(self, machine=machine, quantity=quantity, currency=currency)

    def to_setup_sheet(self, path: Optional[str] = None, economics=None):
        """Return or save a shop-floor setup sheet.

        If *path* is provided, writes JSON to that path and returns the sheet
        dict. Without *path*, returns the sheet dict directly.
        """
        sheet = self._plan.setup_sheet_dict(economics=economics)
        if path:
            import json

            with open(path, "w", encoding="utf-8") as fh:
                json.dump(sheet, fh, indent=2)
        return sheet

    def save_setup_sheet(self, path: str) -> dict:
        """Save the setup sheet JSON to *path* and return the sheet dict."""
        return self.to_setup_sheet(path)

    def export_setup_sheet(self, path: str) -> dict:
        """Compatibility alias for save_setup_sheet."""
        return self.save_setup_sheet(path)

    def setup_sheet_markdown(self, economics=None) -> str:
        """Return a Markdown setup sheet."""
        return self._plan.setup_sheet_markdown(economics=economics)

    def gcode_preview(self) -> str:
        """Return preview-only G-code-like text from neutral toolpaths."""
        return self._plan.gcode_preview()

    def to_gcode_preview(self) -> str:
        """Compatibility alias for gcode_preview."""
        return self.gcode_preview()

    def postprocess(
        self,
        controller: str = "generic_3axis_debug",
        machine: str = "generic_3axis_mill",
        safe_mode: bool = True,
    ) -> str:
        """Return debug-only postprocessor output from the process plan."""
        return self._plan.postprocess(
            controller=controller,
            machine=machine,
            safe_mode=safe_mode,
        )

    def compare_to_target(self, target, tolerance_mm: float = 0.25) -> dict:
        """Compare this stock geometry against a target mesh/STEP/STL/Stock."""
        from .target_compare import compare_to_target

        return compare_to_target(self, target, tolerance_mm=tolerance_mm)

    def visualization_package(
        self,
        output_dir: str,
        *,
        target=None,
        tolerance_mm: float = 0.25,
        include_diff_mesh: bool = True,
        economics=None,
    ) -> dict:
        """Export browser-friendly visualization assets into a directory."""
        from .visualization import export_visualization_package

        return export_visualization_package(
            self,
            output_dir,
            target=target,
            tolerance_mm=tolerance_mm,
            include_diff_mesh=include_diff_mesh,
            economics=economics,
        )

    def simulation_timeline(
        self,
        *,
        fps: int = 30,
        playback_rate: float = 1.0,
        resolution_mm: float = 0.5,
        keyframe_interval_sec: float = 0.25,
        target=None,
        include_machine: bool = True,
        include_fixtures: bool = True,
        include_process_effects: bool = True,
        max_keyframes: int = 240,
    ) -> dict:
        """Return a time-based operation simulation timeline.

        The returned keyframes are visual simulation states, not CadQuery/STEP
        B-Rep states.  Final CAD output remains unchanged.
        """
        from .operation_video import build_simulation_timeline

        return build_simulation_timeline(
            self,
            fps=fps,
            playback_rate=playback_rate,
            resolution_mm=resolution_mm,
            keyframe_interval_sec=keyframe_interval_sec,
            target=target,
            include_machine=include_machine,
            include_fixtures=include_fixtures,
            include_process_effects=include_process_effects,
            max_keyframes=max_keyframes,
        )

    def export_simulation_package(
        self,
        output_dir: str,
        *,
        target=None,
        tolerance_mm: float = 0.25,
        include_diff_mesh: bool = True,
        economics=None,
        fps: int = 30,
        playback_rate: float = 1.0,
        resolution_mm: float = 0.5,
        keyframe_interval_sec: float = 0.25,
        include_machine: bool = True,
        include_fixtures: bool = True,
        include_process_effects: bool = True,
        max_keyframes: int = 240,
    ) -> dict:
        """Export a browser package with operation-video timeline/keyframes."""
        from .operation_video import export_simulation_package

        return export_simulation_package(
            self,
            output_dir,
            target=target,
            tolerance_mm=tolerance_mm,
            include_diff_mesh=include_diff_mesh,
            economics=economics,
            fps=fps,
            playback_rate=playback_rate,
            resolution_mm=resolution_mm,
            keyframe_interval_sec=keyframe_interval_sec,
            include_machine=include_machine,
            include_fixtures=include_fixtures,
            include_process_effects=include_process_effects,
            max_keyframes=max_keyframes,
        )

    def validate_shop_floor(self, structured: bool = False, raise_on_error: bool = False):
        """Run shop-floor validation against the current process plan."""
        from .validation import validate_all

        report = validate_all(
            self._plan.to_dict(),
            self._stock_dims,
            structured=structured or raise_on_error,
        )
        if raise_on_error and getattr(report, "errors", None):
            raise ValueError("; ".join(report.errors))
        return report

    def assert_shop_floor_valid(self) -> None:
        """Raise ``ValueError`` when the current machine/process plan has errors."""
        self.validate_shop_floor(structured=True, raise_on_error=True)

    def plan_summary(self) -> str:
        """Return a human-readable summary of the process plan."""
        return self._plan.summary()

    @property
    def volume(self) -> float:
        """Current volume of the workpiece in mm^3."""
        return compute_volume(self._shape)

    @property
    def bounding_box(self) -> dict:
        """Current bounding box of the workpiece."""
        return compute_bounding_box(self._shape)

    @property
    def material(self) -> str:
        """Workpiece material key."""
        return self._material

    def __repr__(self) -> str:
        bbox = self.bounding_box
        ops = self._plan.total_operations
        return (
            f"Stock(material={self._material!r}, "
            f"dims={bbox.get('length', 0):.0f}x{bbox.get('width', 0):.0f}x{bbox.get('height', 0):.0f} mm, "
            f"ops={ops}, volume={self.volume:.0f} mm^3)"
        )


# =============================================================================
#  Simulation helper (module-level)
# =============================================================================

def _serialize_toolpath(toolpath):
    """Convert operation toolpath objects into JSON-serializable records."""
    if hasattr(toolpath, "to_dict"):
        return toolpath.to_dict()

    serialized = []
    for pose in toolpath:
        if hasattr(pose, "to_dict"):
            serialized.append(pose.to_dict())
            continue

        if isinstance(pose, dict):
            serialized.append(dict(pose))
            continue

        position = getattr(pose, "position", None)
        orientation = getattr(pose, "orientation", None)
        if position is not None and orientation is not None:
            serialized.append({
                "position": list(position),
                "orientation": list(orientation),
            })
            continue

        if isinstance(pose, (list, tuple)) and len(pose) >= 3:
            serialized.append({
                "position": [pose[0], pose[1], pose[2]],
                "orientation": [1.0, 0.0, 0.0, 0.0],
            })

    return serialized


def _toolpath_from_serialized(toolpath_data, *, z_offset: float = 0.0) -> list:
    """Rehydrate serialized operation toolpath records for simulation."""
    try:
        from src.simulation.kinematics import ToolPose
    except ImportError:
        from simulation.kinematics import ToolPose

    import numpy as np

    if isinstance(toolpath_data, dict):
        if toolpath_data.get("metadata", {}).get("coordinate_system") == "part_mm":
            z_offset = 0.0
        if "moves" in toolpath_data:
            toolpath_data = toolpath_data["moves"]
        elif "positions" in toolpath_data:
            toolpath_data = toolpath_data["positions"]
        else:
            toolpath_data = [toolpath_data]

    toolpath = []
    for record in toolpath_data:
        if hasattr(record, "position") and hasattr(record, "orientation"):
            toolpath.append(record)
            continue

        if isinstance(record, dict):
            position = record.get("position")
            orientation = record.get("orientation", [1.0, 0.0, 0.0, 0.0])
        elif isinstance(record, (list, tuple)) and len(record) >= 3:
            position = record[:3]
            orientation = [1.0, 0.0, 0.0, 0.0]
        else:
            continue

        if position is None:
            continue

        position = list(position)
        if len(position) >= 3:
            position[2] = float(position[2]) + float(z_offset or 0.0)

        toolpath.append(ToolPose(
            position=np.array(position, dtype=np.float64),
            orientation=np.array(orientation, dtype=np.float64),
        ))

    return toolpath


def _op_to_simple_toolpath(op_dict: dict, bbox: dict) -> list:
    """Generate a minimal toolpath for simulation from an operation dict.

    Returns a list of ToolPose objects in the stock coordinate frame.
    Each pose positions the tool tip at (cx, cy, z) with a vertical
    downward orientation (tool axis = [0, 0, 1] in world, cutting -Z).

    For face_mill and chamfer operations, generates a raster/contour
    pattern across the stock top face.  For pocket/drill/slot operations,
    generates a single plunge at the operation position.
    """
    authored_toolpath = op_dict.get("toolpath")
    if authored_toolpath:
        preferred = _toolpath_from_serialized(
            authored_toolpath,
            z_offset=float(bbox.get("z_max", 0.0) or 0.0),
        )
        if preferred:
            return preferred

    try:
        from src.simulation.kinematics import ToolPose
    except ImportError:
        from simulation.kinematics import ToolPose

    import numpy as np

    op_type = op_dict.get("operation", "")
    depth = op_dict.get("depth_mm", 0.0)
    pos = op_dict.get("position")
    cx, cy = pos if pos else (0.0, 0.0)
    z_top = bbox.get("z_max", 0.0)

    # Default orientation: tool pointing +Z in world, cutting is -Z
    q_identity = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    # Sanity check: depth must be positive
    if depth <= 0.0:
        depth = 1.0

    toolpath: list = []

    if op_type in ("face_mill", "chamfer"):
        # Face mill / chamfer: raster across the top face
        x_min = bbox.get("x_min", 0.0)
        x_max = bbox.get("x_max", 0.0)
        y_min = bbox.get("y_min", 0.0)
        y_max = bbox.get("y_max", 0.0)
        tool_dia = op_dict.get("tool_diameter_mm", 10.0)
        stepover = tool_dia * 0.7

        # Generate raster passes in X with stepover in Y
        steps = max(1, int(depth / 0.5))
        y = y_min
        direction = 1  # alternating
        while y < y_max:
            z = z_top - depth
            toolpath.append(ToolPose(
                position=np.array([x_min, y, z], dtype=np.float64),
                orientation=q_identity.copy(),
            ))
            toolpath.append(ToolPose(
                position=np.array([x_max, y, z], dtype=np.float64),
                orientation=q_identity.copy(),
            ))
            y += stepover
            direction *= -1
    else:
        # Pocket, drill, slot, contour, etc.: plunge at operation position
        steps = max(1, int(depth / 0.5))  # 0.5mm steps
        for i in range(steps):
            z = z_top - (i + 1) * (depth / steps)
            toolpath.append(ToolPose(
                position=np.array([cx, cy, z], dtype=np.float64),
                orientation=q_identity.copy(),
            ))

    return toolpath


# =============================================================================
#  Self-test
# =============================================================================

if __name__ == "__main__":
    print("=== Stock Self-Test ===\n")

    if not _HAS_CADQUERY:
        print("SKIP: cadquery not installed. Tests require cadquery.")
        exit(0)

    import tempfile
    import os

    passed = 0
    failed = 0

    def _check(label: str, condition: bool, detail: str = ""):
        global passed, failed
        if condition:
            passed += 1
            print(f"  PASS  {label}")
        else:
            failed += 1
            print(f"  FAIL  {label}  -- {detail}")

    # ------------------------------------------------------------------
    # 1. Factory constructors
    # ------------------------------------------------------------------
    print("1. Factory constructors ...")

    s1 = Stock.rectangular(100, 50, 20, material="aluminum_6061")
    _check("rectangular stock created", s1 is not None)
    _check("material", s1._material == "aluminum_6061")
    _check("plan has 0 ops", s1._plan.total_operations == 0)

    bb = s1.bounding_box
    _check("bbox length ~100", abs(bb["length"] - 100.0) < 1.0)
    _check("bbox width ~50", abs(bb["width"] - 50.0) < 1.0)
    _check("bbox height ~20", abs(bb["height"] - 20.0) < 1.0)

    vol_before = s1.volume
    _check("volume > 0", vol_before > 0, f"got {vol_before}")

    s_cyl = Stock.cylindrical(50, 30, "steel_4140")
    _check("cylindrical stock", s_cyl is not None)
    _check("cyl material", s_cyl._material == "steel_4140")
    print()

    # ------------------------------------------------------------------
    # 2. Face mill
    # ------------------------------------------------------------------
    print("2. Face mill ...")

    s2 = s1.face_mill(depth=1.0)
    _check("returns Stock", isinstance(s2, Stock))
    _check("immutable (s1 unchanged)",
           s1._plan.total_operations == 0 and s2._plan.total_operations == 1)
    _check("volume decreased", s2.volume < s1.volume,
           f"s1={s1.volume:.0f}, s2={s2.volume:.0f}")

    ops = s2.process_plan()["operations"]
    _check("recorded as face_mill", ops[0]["operation"] == "face_mill")
    _check("depth 1.0", abs(ops[0].get("depth_mm", 0) - 1.0) < 0.01)
    _check("feeds_speeds present", ops[0].get("feeds_speeds") is not None)
    print()

    # ------------------------------------------------------------------
    # 3. Pocket
    # ------------------------------------------------------------------
    print("3. Pocket ...")

    s3 = s2.pocket(width=30, length=20, depth=5, corner_radius=0)
    _check("returns Stock", isinstance(s3, Stock))
    _check("2 ops total", s3._plan.total_operations == 2)
    _check("volume decreased", s3.volume < s2.volume)

    pk_ops = [o for o in s3.process_plan()["operations"]
              if o["operation"] == "rough_pocket"]
    _check("recorded as rough_pocket", len(pk_ops) == 1,
           f"got {[o['operation'] for o in s3.process_plan()['operations']]}")
    print()

    # ------------------------------------------------------------------
    # 4. Drill
    # ------------------------------------------------------------------
    print("4. Drill ...")

    s4 = s3.drill(diameter=6, cx=25, cy=10, depth=15, spot_drill=False)
    _check("returns Stock", isinstance(s4, Stock))
    _check("3 ops total", s4._plan.total_operations == 3)

    drill_ops = [o for o in s4.process_plan()["operations"]
                 if o["operation"] == "drill"]
    _check("recorded in plan", len(drill_ops) == 1)
    _check("volume decreased", s4.volume < s3.volume)
    print()

    # ------------------------------------------------------------------
    # 5. Chamfer
    # ------------------------------------------------------------------
    print("5. Chamfer ...")

    s5 = s4.chamfer(width=0.5)
    _check("returns Stock", isinstance(s5, Stock))
    _check("4 ops total", s5._plan.total_operations == 4)

    chamfer_ops = [o for o in s5.process_plan()["operations"]
                   if o["operation"] == "chamfer"]
    _check("recorded in plan", len(chamfer_ops) == 1)
    print()

    # ------------------------------------------------------------------
    # 6. Threaded hole (compound)
    # ------------------------------------------------------------------
    print("6. Threaded hole ...")

    s6 = s2.threaded_hole(diameter=6, depth=12, cx=15, cy=15)
    _check("returns Stock", isinstance(s6, Stock))
    _check("spot+drill+tap = +3 ops",
           s6._plan.total_operations == s2._plan.total_operations + 3,
           f"expected {s2._plan.total_operations + 3}, got {s6._plan.total_operations}")

    th_ops = s6.process_plan()["operations"][-3:]
    th_names = [o["operation"] for o in th_ops]
    _check("spot_drill first", th_names[0] == "spot_drill", f"got {th_names}")
    _check("drill second", th_names[1] == "drill", f"got {th_names}")
    _check("tap third", th_names[2] == "tap", f"got {th_names}")
    print()

    # ------------------------------------------------------------------
    # 7. Full chain
    # ------------------------------------------------------------------
    print("7. Full chain ...")

    s_full = (Stock.rectangular(100, 50, 20, material="aluminum_6061")
        .face_mill(depth=1.0)
        .pocket(width=30, length=20, depth=5, corner_radius=2)
        .drill(diameter=6, cx=25, cy=10, depth=15)
        .chamfer(width=0.5)
    )
    _check("5+ ops", s_full._plan.total_operations >= 5,
           f"got {s_full._plan.total_operations}")
    _check("volume < original", s_full.volume < s1.volume)
    _check("process_plan returns dict", isinstance(s_full.process_plan(), dict))
    _check("plan has operations", "operations" in s_full.process_plan())
    print()

    # ------------------------------------------------------------------
    # 8. Immutability
    # ------------------------------------------------------------------
    print("8. Immutability ...")

    s_a = Stock.rectangular(50, 50, 10)
    s_b = s_a.face_mill(1.0)
    s_c = s_b.pocket(20, 20, 3)

    _check("s_a unchanged", s_a._plan.total_operations == 0)
    _check("s_b unchanged", s_b._plan.total_operations == 1)
    _check("s_c has both", s_c._plan.total_operations == 2)
    print()

    # ------------------------------------------------------------------
    # 9. Export
    # ------------------------------------------------------------------
    print("9. Export ...")

    tmp_step = tempfile.NamedTemporaryFile(suffix=".step", delete=False)
    tmp_step_path = tmp_step.name
    tmp_step.close()
    try:
        s_full.to_step(tmp_step_path)
        size = os.path.getsize(tmp_step_path)
        _check(f"STEP export: {size} bytes", size > 100)
    finally:
        os.unlink(tmp_step_path)

    tmp_json = tempfile.NamedTemporaryFile(suffix=".json", delete=False)
    tmp_json_path = tmp_json.name
    tmp_json.close()
    try:
        s_full.save_process_plan(tmp_json_path)
        size = os.path.getsize(tmp_json_path)
        _check(f"Plan JSON export: {size} bytes", size > 100)
    finally:
        os.unlink(tmp_json_path)

    _check("plan_summary returns str", isinstance(s_full.plan_summary(), str))
    _check("plan_summary non-empty", len(s_full.plan_summary()) > 0)
    _check("__repr__ works", "Stock(" in repr(s_full))
    print()

    # ------------------------------------------------------------------
    # 10. Contour and slot
    # ------------------------------------------------------------------
    print("10. Contour and slot ...")

    s10 = Stock.rectangular(80, 60, 20)
    s10 = s10.contour(depth=5, stepdown=2.0)
    _check("contour: 1 op", s10._plan.total_operations == 1)

    s10 = s10.slot(length=30, width=6, depth=5, cx=10, cy=0)
    _check("slot: 2 ops", s10._plan.total_operations == 2)
    print()

    # ------------------------------------------------------------------
    # 11. from_step
    # ------------------------------------------------------------------
    print("11. from_step ...")

    tmp_step2 = tempfile.NamedTemporaryFile(suffix=".step", delete=False)
    tmp_step2_path = tmp_step2.name
    tmp_step2.close()
    try:
        s_full.to_step(tmp_step2_path)
        s_from = Stock.from_step(tmp_step2_path, material="steel_304", stock_oversize=5.0)
        _check("from_step creates Stock", isinstance(s_from, Stock))
        _check("material is steel_304", s_from._material == "steel_304")
        _check("stock larger than target", s_from.volume > s_full.volume)
    finally:
        os.unlink(tmp_step2_path)
    print()

    # ==================================================================
    #  Summary
    # ==================================================================
    total = passed + failed
    print(f"=== {'ALL TESTS PASSED' if failed == 0 else 'SOME TESTS FAILED'} "
          f"({passed}/{total} passed) ===")
    if failed > 0:
        exit(1)
