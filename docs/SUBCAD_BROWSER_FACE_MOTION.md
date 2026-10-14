# Native face motion for browser simulation

Date: 2026-10-14

The paths cover face milling and rectangular pocket milling on six cardinal stock faces.
The paths use the actual incoming material bounds.
They preserve the stock coordinate origin.

The rectangular pocket length follows local X.
The rectangular pocket width follows local Y.
The cutter-centre contour follows the modeled corner radius minus the cutter radius.
The contour uses linear chords with a maximum error of 0.002 mm.
The final inner contour prevents a remaining material island.

Face milling uses the retained native cutter passes through the full incoming stock footprint.
The native path preserves its entry and cutter positions.
An unbound face plan has no browser stock binding.
A buried pocket has no toolpath through the access face.

Bound paths provide:

- `coordinate_system: part_mm` for retained native paths, or `stock_mm` for the legacy rectangular pocket.
- `tool_origin: cutting_tip`.
- A fixed, right-handed `tool_frame` with three unit axes.
- The incoming material bounds in millimetres.
- The stock face and the clearance distance.
- `source: cam_intent` and `verification_status: not_verified`.

Legacy rectangular pocket paths use `safe_z` as a clearance distance.
Native retained paths keep the native meaning of `safe_z`.
The browser uses the recorded absolute move positions.
The operation-video adapter uses the stock positions and the fixed tool frame.
It does not add the stock top height again.

The regression tests compare cutter removal with native material membership.
The tests cover six faces, nonsquare pockets, larger corner radii, translated stock, and preceding cuts.
An independent exact cylinder and prism sweep also matches the nonsquare pocket BRep.
The operation-video contract passes 46 checks.

These checks do not qualify feeds, fixtures, holders, collisions, machine NC, or customer STEP tolerances.
The native material states remain separate from manufacturing release.
Other Milling operations still need their own coordinate binding and independent cutter replay.
