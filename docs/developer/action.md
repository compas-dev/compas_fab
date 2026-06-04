# ActionChain: design notes and future work

[`ActionChain`][compas_fab.robots.ActionChain] is the backend-agnostic
data container for assembling multi-stage motions out of
[`Action`][compas_fab.robots.Action] objects. The current API supports linear processes, such as pick-and-place, and 3D printing: a chain of trajectory actions interspersed with discrete state-change actions, threaded through a single cell state.

A single `Action` class is used for both kinds — an action with a
trajectory is *planned*, one without is *unplanned* (a discrete state
change). Using one class rather than a planned/unplanned split means
backtracking/undo just clears the trajectory instead of swapping class
types. The name "chain" was chosen because each action's end-state equals
the next action's start-state. (Rejected alternatives: *motion skeleton*,
*template*, *motion plan*, *action sequence*, *action group*.)

This page is for contributors. It records (a) what was intentionally
deferred so that vNext work doesn't accidentally break the public API, and
(b) the forward-compat hooks the current API already has.

## Deferred to vNext: tree / branching chains

A backtracking planner -or any planner that enumerates alternative
solution paths- would naturally produce a **tree** of trajectories
rather than a linear chain:

- At any node a planner could try multiple candidate sub-paths
  (different IK seeds, different intermediate poses, different
  grasps).
- The chain could carry both the *realized* path that was chosen and
  the *alternatives* that were considered.

The current version is linear-only on purpose. There is no concrete
planner today that produces trees, and designing the tree representation
before a real producer exists risks getting the abstraction wrong.

The right time to add tree support is when there is a concrete planner
that needs it. The current API is shaped to keep that addition non-breaking.

## Forward-compat hooks the current API already provides

A future contributor adding tree support should not have to change
public APIs that users already depend on. Three things in the current version
exist specifically to make that true:

### 1. Iteration goes through methods, not a raw list

`__iter__` and `iter_actions()` are defined as iterating **the realized
path**. The `actions` property returns a fresh snapshot list, also along
the realized path; it is not the underlying storage.

```python
for action in chain:              # always: realized path
    ...
for action in chain.iter_actions(): # same
    ...
chain.actions                     # read-only snapshot of the realized path
```

When tree support lands, `iter_actions()` keeps its meaning and a new
`iter_branches()` (or similar) walks the alternatives. Users who wrote
linear-style code keep working.

**Do not** add a `chain.actions = new_list` setter or expose the
underlying storage as a mutable list: that would lock in linear
semantics.

### 2. Action lookup is by name, not by index

`action_by_name(name)` is the only first-class lookup primitive. There is
deliberately no `chain[i]` or `chain.actions_by_index(i)` API. Indexes are
ambiguous in a tree (which branch?); names are unique across the whole
chain by validation on `append_*`.

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
    "actions": [...]
}
```

`actions` is the realized path. When tree support lands, the planned
extension is a sibling key:

```json
{
    "name": "...",
    "start_state": {...},
    "actions": [...],
    "alternatives": {
        "action_name_X": [...sub-chain...],
        "action_name_Y": [...]
    }
}
```

Reading an old (linear) JSON in tree-aware code: `alternatives` is
absent, so the chain degenerates to a single chain.
Reading a tree JSON in linear-only code: the unknown `alternatives` key
is ignored by `__from_data__`, and the realized path comes through
unchanged.

**Do not** repurpose the `actions` key for nested children. Add a new
sibling key instead.

## Open questions for vNext

These need real-world input before being resolved:

- **Branch semantics**: alternatives at a single node, or full
  branching with re-merge? The constrained "alternatives at a node"
  case is much cheaper to design for; allow re-merge only if a real use
  case demands it.
- **Selection semantics**: how does a tree-aware consumer know which
  child is the realized one? A `selected_child` pointer per branching
  node, or an explicit `realized_path: [action_name, ...]` at the chain
  level? The latter is more verbose on disk but easier to read by hand.
- **Iteration over the full tree**: BFS or DFS? Returning a flat list,
  or `(action, depth, branch_id)` triples?
- **Walk APIs**: should there be a `chain.prune(predicate)` to drop
  unrealized branches? A `chain.score(criterion)` to rank alternatives?
  Hold off until a planner needs them.

## When in doubt

If you are about to add a public API to `ActionChain` and you are unsure
whether a tree-aware consumer would still be able to use it correctly,
**don't add it**. The class is small and easy to extend later; a wrong
API addition is much harder to retract.
