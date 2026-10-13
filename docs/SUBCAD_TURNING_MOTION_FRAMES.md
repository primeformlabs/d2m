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
