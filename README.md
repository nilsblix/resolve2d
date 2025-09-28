# Zigics

2D rigid-body realtime physics engine written in Zig, with both WebAssemblw and
native demos. Zigics provides simple primitives, a small constraint set and an
iterative solver with broadphase and narrowphase collision detection. The goal
is to be educational, hackable, lightweight and fast enough for real-time
applications.

Status: early WIP. Expect breaking changes and sharp edges.

## Features
Interfaces have been created for the following three physical concepts:
- Rigidbodies: The solver can simulate the geometry of any convex polygon,
although only disc- and rectangle geometries have been implemented so far.
- Constraints: A constraint is a set of equations that describe how a body
should behave, such as a bead on a string or point fixed in space. The solver
is able to simulate both positional and velocity based constraints, with them
being either equality- or inequality based constraints. The solver is even
equipped with calculating and limiting the power outputted by a single
constraint, which means that constraints such as motors, springs or even
animations are able to be simulated via this pipeline.
- Forces: An abstraction for forces is used, which can be applied to all
bodies, such as gravity, or single-body based ones such as springs.

The engine features more highlights, such as:
- Collisions: Collisions can be represented via the constraint pipeline as a
positional inequality constraint, however it was split from the constraint
pipeline as gathering collision-pairs is easier done outside of the constraint
pipeline.
- Determinism: If the same inputs are presented, then the engine gives the same
result. This is possible due to the solver referring to bodies via identities,
and not relying on pure pointer arithmetic.
- Demos: Native (raylib) and web (WASM + Canvas +
[imgui.ts](https://github.com/nilsblix/imgui.ts).

## Installation
If used in a Zig-project, then simply add it to the target project's build.zig.zon
via `$ zig fetch --save https://github.com/nilsblix/zigics.git`.

If used in a web-environment, then add the zigics-wasm-binary to the target
project and instantiate a WASM module. See demos/web for an example on how to
package the WASM module.

## Quick Start
__TODO__

## Demos
__TODO__

## Roadmap
- Refactor `bodies` property of Solver:
    - Create struct `Bodies`, which contains hashmap of bodies and ids, like
    current implementation in Solver. This struct can contain hashmaps of other
    properties, such as `static`, `sleeping` etc. This will make the program
    more memory efficient and make each `RigidBody` have a smaller memory
    footprint.
    - This will also enable composite/aggregate bodies that distribute impulses
    to member bodies.
- Collision and stability:
    - Persist manifolds across frames by keying with stable body IDs, not
    pointers.
    - Wire solver spatial-hash configuration (cell width/table size) instead of
    hard-coded values.
    - Sleeping/awakening, better stacking stability and optional restitution.
- Rigidbodes, constraints and forces:
    - More Rigidbody implementations, with triangle, star and generalized
    polygon.
    - More joints (hinge/pivot, spring-damper, prismatic/slider).
    - Tunable friction models and force generators.
- WASM and tooling:
    - Expand the bridge API and document exports.
    - Demo UX improvements and more showcase scenes.
- Quality and docs:
    - More tests around collision/constraint edge cases.
    - Clearer examples and API docs; packaging guidance for depending projects.

## Contributing
Issues and PRs are welcome. The codebase aims to stay small and readable;
contributions that keep things simple are appreciated.
