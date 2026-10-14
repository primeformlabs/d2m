# Native nominal comparison

`compare_nominal_solids()` compares the actual candidate solid with the required STEP solid.
The caller must supply an explicit rigid transform from the required STEP frame to the part frame.
The comparison does not infer alignment from equal outer dimensions.

The native engine calculates two Boolean differences:

- Required material that the candidate lacks.
- Stock that the candidate retains outside the required part.

The result preserves both difference solids for a 3D review.
Equal volumes and equal bounding boxes cannot hide different hole positions.
An invalid solid, a whole assembly, or a scaled datum cannot supply part proof.
The STEP reader establishes millimetre units before the transfer.

The numerical volume limit only handles native kernel resolution.
It is not an allowed part tolerance.
`no_resolved_difference` does not qualify a machine, fixture, tool, datum, inspection method, or requested tolerance.
The result always retains `manufacturing_ready: false` and `tolerance_verified: false`.

Twelve checks cover actual differences, unchanged inputs, STEP loading, translation, rotation, invalid datums, and invalid targets.
Complete requested tolerance qualification remains open in issue #5.
