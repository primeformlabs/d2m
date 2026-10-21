"""SubCAD material module -- cleaner API for feeds/speeds and material properties.

Wraps the low-level material database in ``src.knowledge_graph.rules`` to provide
a simplified, stable interface for machining operations in the subtractive CAD library.
"""

from __future__ import annotations

from typing import Optional

# ---------------------------------------------------------------------------
# Import guard -- the rules module may not always be available
# ---------------------------------------------------------------------------

_kg_available = False

try:
    from src.knowledge_graph.rules import (
        get_material_properties,
        list_materials as _kg_list_materials,
        get_compatible_processes as _kg_get_compatible_processes,
        get_feeds_speeds as _kg_get_feeds_speeds,
    )
    _kg_available = True
except ImportError:
    try:
        from knowledge_graph.rules import (
            get_material_properties,
            list_materials as _kg_list_materials,
            get_compatible_processes as _kg_get_compatible_processes,
            get_feeds_speeds as _kg_get_feeds_speeds,
        )
        _kg_available = True
    except ImportError:
        pass


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def get_material_props(material: str) -> dict:
    """Get material properties dict. Returns empty dict if unknown.

    Wraps ``knowledge_graph.rules.get_material_properties``.
    """
    if _kg_available:
        return get_material_properties(material)
    return {}


def list_materials() -> list[str]:
    """Return all known material keys.

    Wraps ``knowledge_graph.rules.list_materials``.
    """
    if _kg_available:
        return _kg_list_materials()
    return []


def get_feeds_speeds(
    tool_type: str,
    tool_diameter_mm: float,
    material: str,
) -> dict:
    """Get feeds and speeds for a tool/material combination.

    Returns dict with keys: tool_type, material, tool_diameter_mm, sfm,
    surface_speed_m_min, fpt_mm, num_flutes, rpm, feed_rate_mm_min.

    Returns dict with ``'error'`` key if the combination is not found, or
    an empty dict if the rules module is unavailable.
    """
    from .program_mode import feeds_speeds_provider
    provider = feeds_speeds_provider()
    if provider is not None:
        # The caller's own cutting data (D2O #729): its tools, its material groups, its machine.
        supplied = provider(tool_type, tool_diameter_mm, material)
        if supplied:
            return dict(supplied)
    if _kg_available:
        return _kg_get_feeds_speeds(tool_type, tool_diameter_mm, material)
    return {}


def get_compatible_processes(material: str) -> list[str]:
    """Get manufacturing processes compatible with a material.

    Wraps ``knowledge_graph.rules.get_compatible_processes``.
    """
    if _kg_available:
        return _kg_get_compatible_processes(material)
    return []


def is_plastic(material: str) -> bool:
    """Check if *material* is a plastic."""
    props = get_material_props(material)
    return props.get("material_type") == "plastic"


def is_metal(material: str) -> bool:
    """Check if *material* is a metal."""
    props = get_material_props(material)
    return props.get("material_type") == "metal"


def get_melting_point(material: str) -> Optional[float]:
    """Get melting point in degrees Celsius, or *None* if unknown."""
    props = get_material_props(material)
    return props.get("melting_point_c")


def get_machinability(material: str) -> Optional[float]:
    """Get machinability percentage (relative to brass C360 = 100%), or *None*."""
    props = get_material_props(material)
    return props.get("machinability_pct")


# =============================================================================
# Self-test
# =============================================================================

if __name__ == "__main__":
    print("=== SubCAD material module self-test ===")
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
    # 1. get_material_props
    # ------------------------------------------------------------------
    print("\n1. get_material_props ...")
    props = get_material_props("aluminum_6061")
    _check("aluminum_6061 exists", len(props) > 0)
    _check("density == 2.70", abs(props.get("density_gcm3", 0) - 2.70) < 0.01)
    _check("material_type is metal", props.get("material_type") == "metal")

    props_abs = get_material_props("abs")
    _check("abs exists", len(props_abs) > 0)
    _check("abs is plastic", props_abs.get("material_type") == "plastic")

    _check("unknown material returns empty",
           get_material_props("unobtanium") == {})

    # ------------------------------------------------------------------
    # 2. list_materials
    # ------------------------------------------------------------------
    print("\n2. list_materials ...")
    mats = list_materials()
    _check("at least 10 materials", len(mats) >= 10,
           f"got {len(mats)}")
    _check("aluminum_6061 present", "aluminum_6061" in mats)
    _check("peek present", "peek" in mats)

    # ------------------------------------------------------------------
    # 3. get_feeds_speeds
    # ------------------------------------------------------------------
    print("\n3. get_feeds_speeds ...")
    fs = get_feeds_speeds("end_mill", 10.0, "aluminum_6061")
    _check("no error key", "error" not in fs,
           f"got error: {fs.get('error', '')}")
    _check("rpm > 0", fs.get("rpm", 0) > 0,
           f"got rpm={fs.get('rpm')}")
    _check("feed_rate_mm_min > 0", fs.get("feed_rate_mm_min", 0) > 0)
    _check("sfm present", "sfm" in fs)
    _check("surface_speed_m_min present", "surface_speed_m_min" in fs)
    _check("fpt_mm present", "fpt_mm" in fs)
    _check("num_flutes present", "num_flutes" in fs)
    _check("tool_type echoed back", fs.get("tool_type") == "end_mill")
    _check("material echoed back", fs.get("material") == "aluminum_6061")
    _check("tool_diameter_mm echoed back",
           abs(fs.get("tool_diameter_mm", 0) - 10.0) < 0.01)

    # Unknown material -> error dict
    fs_err = get_feeds_speeds("end_mill", 10.0, "nonsense_material")
    _check("unknown material returns error key", "error" in fs_err)

    # Unknown tool type -> error dict
    fs_err2 = get_feeds_speeds("laser_cutter", 10.0, "aluminum_6061")
    _check("unknown tool returns error key", "error" in fs_err2)

    # ------------------------------------------------------------------
    # 4. is_plastic / is_metal
    # ------------------------------------------------------------------
    print("\n4. is_plastic / is_metal ...")
    _check("aluminum_6061 is metal", is_metal("aluminum_6061"))
    _check("aluminum_6061 is not plastic", not is_plastic("aluminum_6061"))
    _check("steel_4140 is metal", is_metal("steel_4140"))
    _check("steel_4140 is not plastic", not is_plastic("steel_4140"))
    _check("titanium_6al4v is metal", is_metal("titanium_6al4v"))
    _check("brass_c360 is metal", is_metal("brass_c360"))

    _check("abs is plastic", is_plastic("abs"))
    _check("abs is not metal", not is_metal("abs"))
    _check("polycarbonate is plastic", is_plastic("polycarbonate"))
    _check("nylon_pa6 is plastic", is_plastic("nylon_pa6"))
    _check("peek is plastic", is_plastic("peek"))
    _check("polypropylene is plastic", is_plastic("polypropylene"))

    # ------------------------------------------------------------------
    # 5. get_melting_point
    # ------------------------------------------------------------------
    print("\n5. get_melting_point ...")
    mp_al = get_melting_point("aluminum_6061")
    _check("aluminum_6061 melting point is numeric", isinstance(mp_al, (int, float)))
    _check("aluminum_6061 melting point ~585 C",
           mp_al is not None and abs(mp_al - 585) < 10,
           f"got {mp_al}")

    mp_peek = get_melting_point("peek")
    _check("peek melting point is numeric", isinstance(mp_peek, (int, float)))
    _check("peek melting point ~343 C",
           mp_peek is not None and abs(mp_peek - 343) < 10,
           f"got {mp_peek}")

    mp_unknown = get_melting_point("unobtanium")
    _check("unknown material returns None", mp_unknown is None)

    # ------------------------------------------------------------------
    # 6. get_machinability
    # ------------------------------------------------------------------
    print("\n6. get_machinability ...")
    mach_al = get_machinability("aluminum_6061")
    _check("aluminum_6061 machinability is numeric", isinstance(mach_al, (int, float)))
    _check("aluminum_6061 machinability ~270",
           mach_al is not None and abs(mach_al - 270) < 10,
           f"got {mach_al}")

    mach_brass = get_machinability("brass_c360")
    _check("brass_c360 machinability ~100 (baseline)",
           mach_brass is not None and abs(mach_brass - 100) < 10,
           f"got {mach_brass}")

    mach_abs = get_machinability("abs")
    _check("abs machinability is None (not rated for plastics)",
           mach_abs is None)

    mach_unknown = get_machinability("unobtanium")
    _check("unknown material machinability is None", mach_unknown is None)

    # ------------------------------------------------------------------
    # 7. get_compatible_processes
    # ------------------------------------------------------------------
    print("\n7. get_compatible_processes ...")
    procs_al = get_compatible_processes("aluminum_6061")
    _check("aluminum_6061 has processes", len(procs_al) > 0)
    _check("aluminum_6061 supports milling", "milling" in procs_al)
    _check("aluminum_6061 supports drilling", "drilling" in procs_al)

    procs_abs = get_compatible_processes("abs")
    _check("abs supports injection_molding", "injection_molding" in procs_abs)
    _check("abs supports 3d_printing_fdm", "3d_printing_fdm" in procs_abs)

    procs_unknown = get_compatible_processes("unobtanium")
    _check("unknown material returns empty list", procs_unknown == [])

    # ------------------------------------------------------------------
    # 8. Graceful handling of unknown materials (no crashes)
    # ------------------------------------------------------------------
    print("\n8. Unknown-material gracefulness ...")

    try:
        _ = get_material_props("garbage123")
        _check("get_material_props unknown: no crash", True)
    except Exception as exc:
        _check("get_material_props unknown: no crash", False, str(exc))

    try:
        _ = is_plastic("garbage123")
        _check("is_plastic unknown: no crash", True)
    except Exception as exc:
        _check("is_plastic unknown: no crash", False, str(exc))

    try:
        _ = is_metal("garbage123")
        _check("is_metal unknown: no crash", True)
    except Exception as exc:
        _check("is_metal unknown: no crash", False, str(exc))

    try:
        _ = get_melting_point("garbage123")
        _check("get_melting_point unknown: no crash", True)
    except Exception as exc:
        _check("get_melting_point unknown: no crash", False, str(exc))

    try:
        _ = get_machinability("garbage123")
        _check("get_machinability unknown: no crash", True)
    except Exception as exc:
        _check("get_machinability unknown: no crash", False, str(exc))

    try:
        _ = get_feeds_speeds("end_mill", 10.0, "garbage123")
        _check("get_feeds_speeds unknown material: no crash", True)
    except Exception as exc:
        _check("get_feeds_speeds unknown material: no crash", False, str(exc))

    try:
        _ = get_compatible_processes("garbage123")
        _check("get_compatible_processes unknown: no crash", True)
    except Exception as exc:
        _check("get_compatible_processes unknown: no crash", False, str(exc))

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    total = passed + failed
    print(f"\n=== {'ALL TESTS PASSED' if failed == 0 else 'SOME TESTS FAILED'} "
          f"({passed}/{total} passed) ===")
    if failed > 0:
        exit(1)
