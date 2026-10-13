# Explicit Turning spindle motion

Date: 2026-10-14

Turning paths now export a rigid, column-major stock-to-spindle transform.
The transform matches the native geometry transform on six signed cardinal axes.
The radial plane uses positive local X.
Local Z follows the spindle axis.

The path retains the actual incoming spindle bounds.
It retains the native world positions and spindle radius and axial coordinates.

The caller can supply these planned values through `ToolSpec.preferred_feeds_speeds`:

- `rpm` and `feed_rate_mm_min`.
- `spindle_direction`: `+1` or `-1` in the spindle frame.
- `spindle_phase_start_radians`: the phase at the first recorded position.

The exporter records phase at every move when all these values are available.
It uses the recorded motion distance, feed, and rapid rate.
Missing values produce `spindle_phase_status: unavailable`.
The exporter does not assign a default phase.

The thread feed follows the supplied spindle direction and thread hand.
A right-hand thread uses `dZ/dPhase = -pitch / (2π)`.
A left-hand thread uses the opposite sign.

The paths remain CAM intent.
Roughing passes, tool compensation, fixtures, collisions, and controller NC still need separate verification.
Insert cutting profiles must come from the selected tool geometry.
A holder outline cannot define an insert cutting profile.

## Continuous program clock

Call `Stock.process_plan(continuous_turning_phase=True)` to export consecutive Turning paths with one continuous spindle clock.
The exporter copies the plan and preserves the original operation-level export.
It uses the first path's explicit initial phase.
Later paths retain phase from the preceding path.

The clock requires the same identified insert profile, setup, spindle frame, speed, and rotation direction.
The exporter authors non-cutting rapid transport between the recorded endpoints.
The transport uses radial clearance from the incoming stock bounds and insert profile.
Its collision status remains unverified.
The exported positions and phase include this transport time.

An insert, setup, speed, or direction change requires a recorded transition.
Missing transition data makes the phase unavailable for the remaining Turning block.
A new block after another process needs explicit initial phase inputs.
The clock remains CAM intent with `manufacturing_ready: false`.
