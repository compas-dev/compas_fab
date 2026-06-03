# MotionPlan: design notes and future work

[`MotionPlan`][compas_fab.robots.MotionPlan] is the backend-agnostic
data container for assembling multi-stage motions. The current API supports linear processes, such as pick-and-place, and 3D printing: a chain of trajectories interspersed with discrete state changes, threaded through a single cell state.

This page is for contributors. It records (a) what was intentionally
deferred so that vNext work doesn't accidentally break the public API, and
(b) the forward-compat hooks the current API already has.

## Deferred to vNext: tree / branching plans

A backtracking planner -or any planner that enumerates alternative
solution paths- would naturally produce a **tree** of trajectories
rather than a linear chain:

- At any node a planner could try multiple candidate sub-paths
  (different IK seeds, different intermediate poses, different
  grasps).
- The plan could carry both the *realized* path that was chosen and
  the *alternatives* that were considered.

THe current version is linear-only on purpose. There is no concrete
planner today that produces trees, and designing the tree representation
before a real producer exists risks getting the abstraction wrong.

The right time to add tree support is when there is a concrete planner
that needs it. The current API is shaped to keep that addition non-breaking.

## Forward-compat hooks the current API already provides

A future contributor adding tree support should not have to change
public APIs that users already depend on. Three things in the current version
exist specifically to make that true:

### 1. Iteration goes through methods, not a raw list

`__iter__` and `iter_steps()` are defined as iterating **the realized
path**. The `steps` property returns a fresh snapshot list, also along
the realized path; it is not the underlying storage.

```python
for step in plan:              # always: realized path
    ...
for step in plan.iter_steps(): # same
    ...
plan.steps                     # read-only snapshot of the realized path
```

When tree support lands, `iter_steps()` keeps its meaning and a new
`iter_branches()` (or similar) walks the alternatives. Users who wrote
linear-style code keep working.

**Do not** add a `plan.steps = new_list` setter or expose the
underlying storage as a mutable list: that would lock in linear
semantics.

### 2. Step lookup is by name, not by index

`step_by_name(name)` is the only first-class lookup primitive. There is
deliberately no `plan[i]` or `plan.steps_by_index(i)` API. Indexes are
ambiguous in a tree (which branch?); names are unique across the whole
plan by validation on `append_*`.

If you find yourself reaching for an index-based API, prefer iterating
or thinking about names. New public APIs should also take a name, not
an index.

### 3. Serialization shape has room for `alternatives` without a bump

The current top-level data shape is:

```json
{
    "name": "...",
    "description": "...",
    "start_state": {...},
    "cell_signature": "...",
    "steps": [...]
}
```

`steps` is the realized path. When tree support lands, the planned
extension is a sibling key:

```json
{
    "name": "...",
    "start_state": {...},
    "steps": [...],
    "alternatives": {
        "step_name_X": [...sub-chain...],
        "step_name_Y": [...]
    }
}
```

Reading an old (linear) JSON in tree-aware code: `alternatives` is
absent, so the plan degenerates to a single chain.
Reading a tree JSON in linear-only code: the unknown `alternatives` key
is ignored by `__from_data__`, and the realized path comes through
unchanged.

**Do not** repurpose the `steps` key for nested children. Add a new
sibling key instead.

## Open questions for vNext

These need real-world input before being resolved:

- **Branch semantics**: alternatives at a single node, or full
  branching with re-merge? The constrained "alternatives at a node"
  case is much cheaper to design for; allow re-merge only if a real use
  case demands it.
- **Selection semantics**: how does a tree-aware consumer know which
  child is the realized one? A `selected_child` pointer per branching
  node, or an explicit `realized_path: [step_name, ...]` at the plan
  level? The latter is more verbose on disk but easier to read by hand.
- **Iteration over the full tree**: BFS or DFS? Returning a flat list,
  or `(step, depth, branch_id)` triples?
- **Walk APIs**: should there be a `plan.prune(predicate)` to drop
  unrealized branches? A `plan.score(criterion)` to rank alternatives?
  Hold off until a planner needs them.

## When in doubt

If you are about to add a public API to `MotionPlan` and you are unsure
whether a tree-aware consumer would still be able to use it correctly,
**don't add it**. The class is small and easy to extend later; a wrong
API addition is much harder to retract.
