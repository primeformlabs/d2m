"""SubCAD -- Subtractive CAD library.

Start with a stock block and cut material away using machining operations.
Each operation is simultaneously a geometry modifier AND a manufacturing
instruction recorded in a JSON-serializable process plan.

Usage::

    from src.subcad import Stock

    part = (Stock.rectangular(100, 50, 20, material="aluminum_6061")
        .face_mill(depth=1.0)
        .pocket(width=30, length=20, depth=5, corner_radius=2)
        .drill(diameter=6, cx=25, cy=10, depth=15)
        .chamfer(width=0.5)
    )
    part.to_step("output.step")
    print(part.process_plan())
"""

from .stock import Stock
from .process_plan import ProcessPlan, OperationStep
from .machine_context import MachineContext, load_machine_context
from .controller_context import ControllerContext, load_controller_context
from .setup_context import SetupContext, load_setup_context
from .tooling_context import (
    ToolAssembly as ToolingAssembly,
    ToolMagazine,
    ToolingContext,
    load_tooling_context,
    required_tool_types,
)
from .quality_context import (
    QualityContext,
    load_quality_context,
    load_collision_policy,
)
from .tool_library import ToolSpec, ToolAssembly, ToolCatalog, to_sim_tool
from .tool_selection import ToolRequirement, ToolSelectionResult, ToolUnavailableError, select_tool, tool_inventory
from .toolpath import Toolpath, ToolpathMove
from .gcode_preview import gcode_preview, to_gcode_preview
from .postprocessor import PostProcessor, Generic3AxisDebugPost, postprocess
from .target_compare import compare_to_target
from .nominal_compare import compare_nominal_solids, load_nominal_step, NominalComparison
from .laser_material import replay_laser_material, release_laser_part, laser_selection_region, LaserMaterialReplay, LaserReleasedPart
from .milling_verification import replay_milling_independently
from .visualization import export_visualization_package
from .operation_video import export_simulation_package
from .economics import estimate_cost, compare_programs
from .operations import (
    MachiningOperation,
    FaceMillOp,
    PocketOp,
    CircularPocketOp,
    SpotDrillOp,
    DrillOp,
    TapOp,
    ChamferOp,
    ContourOp,
    SlotOp,
    PeckDrillOp,
    ReamOp,
    BoreOp,
    CountersinkOp,
    CounterboreOp,
    ThreadMillOp,
    TSlotOp,
    DovetailOp,
    GrooveOp,
    Surface3DOp,
    DeburrOp,
    SpotFaceOp,
    EdgeChamferOp,
    EdgeFilletOp,
    ProfilePocketOp,
    ProfileCutoutOp,
    ProfileContourOp,
    LaserCutProfileOp,
    LaserCutInternalOp,
    PressBrakeBendOp,
    MachineAroundProfileOp,
    MachineAroundProfilesOp,
    MachineAroundCylinderOp,
    RibOp,
    PadOp,
    TurnProfileOp,
    TurnFaceOp,
    TurnODOp,
    TurnIDOp,
    TurnGrooveOp,
    TurnThreadOp,
    MillTurnSetupOp,
    ThinWallPocketOp,
    HollowBoreOp,
    TubeProfileOp,
    SurfaceMillOp,
    SweepMillOp,
    LoftMillOp,
    DomeMillOp,
    WireCutProfileOp,
    WireCutInternalOp,
    create_operation,
    threaded_hole_ops,
)
from .material import (
    get_material_props,
    list_materials,
    get_feeds_speeds,
    get_compatible_processes,
    is_plastic,
    is_metal,
    get_melting_point,
    get_machinability,
)
from .fixturing import (
    FixtureType,
    FixtureSpec,
    FixtureCatalog,
    ClampingZone,
    Setup,
    fixture_to_mesh,
    fixture_visualization_payload,
    check_operation_against_fixture,
    WORK_OFFSET_NAMES,
)


def build_video_export_plan(*args, **kwargs):
    from .video_export import build_video_export_plan as _build_video_export_plan

    return _build_video_export_plan(*args, **kwargs)


def export_video(*args, **kwargs):
    from .video_export import export_video as _export_video

    return _export_video(*args, **kwargs)


__all__ = [
    # Core classes
    "Stock",
    "ProcessPlan",
    "OperationStep",
    "MachineContext",
    "load_machine_context",
    "ControllerContext",
    "load_controller_context",
    "SetupContext",
    "load_setup_context",
    "ToolingAssembly",
    "ToolMagazine",
    "ToolingContext",
    "load_tooling_context",
    "required_tool_types",
    "QualityContext",
    "load_quality_context",
    "load_collision_policy",
    "ToolSpec",
    "ToolAssembly",
    "ToolCatalog",
    "ToolRequirement",
    "ToolSelectionResult",
    "select_tool",
    "Toolpath",
    "ToolpathMove",
    "gcode_preview",
    "to_gcode_preview",
    "PostProcessor",
    "Generic3AxisDebugPost",
    "postprocess",
    "compare_to_target",
    "compare_nominal_solids", "load_nominal_step", "NominalComparison",
    "replay_laser_material", "release_laser_part", "laser_selection_region", "LaserMaterialReplay", "LaserReleasedPart",
    "replay_milling_independently",
    "export_visualization_package",
    "export_simulation_package",
    "build_video_export_plan",
    "export_video",
    "estimate_cost",
    "compare_programs",
    # Operation classes
    "MachiningOperation",
    "FaceMillOp",
    "PocketOp",
    "CircularPocketOp",
    "SpotDrillOp",
    "DrillOp",
    "TapOp",
    "ChamferOp",
    "ContourOp",
    "SlotOp",
    "PeckDrillOp",
    "ReamOp",
    "BoreOp",
    "CountersinkOp",
    "CounterboreOp",
    "ThreadMillOp",
    "TSlotOp",
    "DovetailOp",
    "GrooveOp",
    "Surface3DOp",
    "DeburrOp",
    "SpotFaceOp",
    "EdgeChamferOp",
    "EdgeFilletOp",
    "ProfilePocketOp",
    "ProfileCutoutOp",
    "ProfileContourOp",
    "LaserCutProfileOp",
    "LaserCutInternalOp",
    "PressBrakeBendOp",
    "MachineAroundProfileOp",
    "MachineAroundProfilesOp",
    "MachineAroundCylinderOp",
    "RibOp",
    "PadOp",
    "TurnProfileOp",
    "TurnFaceOp",
    "TurnODOp",
    "TurnIDOp",
    "TurnGrooveOp",
    "TurnThreadOp",
    "MillTurnSetupOp",
    "ThinWallPocketOp",
    "HollowBoreOp",
    "TubeProfileOp",
    "SurfaceMillOp",
    "SweepMillOp",
    "LoftMillOp",
    "DomeMillOp",
    "WireCutProfileOp",
    "WireCutInternalOp",
    # Factory & helpers
    "create_operation",
    "threaded_hole_ops",
    "to_sim_tool",
    # Material API
    "get_material_props",
    "list_materials",
    "get_feeds_speeds",
    "get_compatible_processes",
    "is_plastic",
    "is_metal",
    "get_melting_point",
    "get_machinability",
    # Fixturing API
    "FixtureType",
    "FixtureSpec",
    "FixtureCatalog",
    "ClampingZone",
    "Setup",
    "fixture_to_mesh",
    "fixture_visualization_payload",
    "check_operation_against_fixture",
    "WORK_OFFSET_NAMES",
]
