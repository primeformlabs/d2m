# Caller selection policies

`tool_inventory(..., selector=callback)` accepts a policy for feasible tools.
The callback receives the `ToolRequirement` and a tuple of candidate `ToolSpec` copies.
It must return an unchanged candidate.

SubCAD checks type, diameter, flute length, reach, availability, and drill-point geometry before it calls the policy.
The policy cannot introduce a tool or change its dimensions.
The context owns the policy, so concurrent callers can use different policies.
Without a callback, SubCAD keeps its existing diameter selection.

D2O uses this interface for owned cutters first, then catalogue cutters ranked by machining cost and tool wear.
Catalogue assemblies remain marked `owned: false`.
The production caller supplies physical inventory only.
