# zigics

## WIP (very much)

Some things to keep in mind:

When building scenes, if one is creating a constraint, "input body" has to be
fresh. I.e, the statement declaring constraint has to come after the creation
of the body. If this is not met, the system will seg-fault. Something to do
with zig being annoying with `array[last_id]`.

WASM:
Wow. This is incredible. Just do `zig build` and run web/index.html via an http server. Wasm is amazing.

Roadmap:
Composite bodies:
* Another list/map/array in the base of solver. 
    key = Arraylist(RigidBody.Id)?
* New struct? `KinematicComposite`?
* When collision:
    Do not apply to base body, apply to the composite. Scaled ofcourse.
* In RigidBody, instead of `static: bool` have `mode: enum...`. Would that solve the collision thing? Not really
    Maybe mode.composite can contain some sort of `contribution` that after
    collision/constraint solving should be applied to the larger body? Tagged enums? Would be good.
* How should I separate static pos/rotation while still complying with composite?
### Seems like I'm in for a rewrite/restructuring of the kinematic system?
I will not redo KinematicProps.
