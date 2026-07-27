# Jolt Buoyancy

Water volumes for the Jolt physics backend. A **Jolt Water Volume** component turns an
entity into a box of water; Jolt rigid bodies overlapping it float, sink or drift
according to their own density and the fluid's settings.

It needs the **JoltPhysics gem** — <https://github.com/jorgetbruno/JoltPhysics/tree/main> —
and has no effect under any other physics backend.

## Why this is a separate gem

Buoyancy is the reference case for the extension-gem pattern: **the physics gem knows
nothing about it.** This one reaches the backend through the public
`JoltPhysicsSystemRequests` bus, and adding water required no change to the physics gem
beyond the accessor that bus already had to expose.

The dividing line that came out of building this, and which soft bodies later tested: a
feature that **owns bodies** belongs inside the physics gem, and one that only
**perturbs bodies someone else owns** can live outside it. Buoyancy is the second kind.
Soft bodies turned out to be the first, and were moved *into* the physics gem
accordingly (see `Gems/JoltSoftBody/MIGRATED.md`); hair is the second kind again and
lives in the `JoltHair` gem, which needs a renderer this gem does not.

## How it reaches Jolt

Two mechanisms, both load-bearing:

- **`Gem::JoltPhysics.API`** is an INTERFACE target that re-exports `Jolt::Jolt` *with
  the compile definitions the physics gem built Jolt with*. Matching those exactly is
  what keeps the modules ABI compatible — a mismatch is an ODR violation that corrupts
  memory rather than failing to link. **Do not link Jolt any other way.**
- **`JoltPhysicsSystemRequests::GetNativePhysicsSystem(sceneHandle)`** returns the
  `JPH::PhysicsSystem*` for scenes the physics gem owns and null otherwise, so this gem
  is inert rather than dangerous under a different backend. Casting a scene's native
  pointer directly would not be safe: `AzPhysics::Scene` exposes `GetNativePointer()`
  but no type identification.

### Jolt's globals are per module

Jolt is statically linked into every module that uses it, so **each module owns its own
copy of Jolt's global allocation hooks, and they start null.** The physics gem
installing *its* copy does nothing for this one.

This cost a crash to learn: entering game mode activated the water volume, which called
`AddStepListener`, whose `push_back` called `JPH::Reallocate` through a null pointer.
`JoltBuoyancyAllocator::Install()` is therefore the first line of **both**
`JoltBuoyancyModule` and `JoltBuoyancyEditorModule`, and it forwards to
`AZ::SystemAllocator` — deliberately identical to the physics gem's, because the
allocations cross the module boundary (this module's `AddStepListener` grows the array;
the physics gem's `~PhysicsSystem` frees it). `JPH::RegisterDefaultAllocator` would
compile and run, and hand the physics gem a malloc block `AZ::SystemAllocator` never
issued.

Note the structural limit: a module that forgets the call still crashes, and no unit
test in another module can catch it. The tests here call `Install()` rather than
`RegisterDefaultAllocator` so there is one allocator story, but keep the call in every
module entry point.

## How the volume works

`JoltWaterVolume` (`Source/Clients/JoltWaterVolume.{h,cpp}`) is the core, and three
things about it are load-bearing:

1. **It is a `JPH::PhysicsStepListener`, not a tick handler.** The impulse has to land
   inside the step it belongs to, at the delta time the solver is about to integrate.
2. **Jolt calls step listeners with every body mutex already held.** Bodies are reached
   through `GetBodyLockInterfaceNoLock()`, and none are added or removed there — using
   `BodyInterface` inside `OnStep` would deadlock. (The soft body class in the physics
   gem does the inverse, and for the same reason: it creates bodies, so it must run
   outside the step and take a real lock.)
3. **The buoyancy factor is derived per body** from its own density (mass ÷ shape
   volume) against the fluid density, so a dense body sinks and a light one bobs. Jolt's
   factor is relative: 1 is neutral, above 1 floats.

The volume's **local +Z face is the water surface**, so a tilted volume gives a tilted
surface. Settings are snapshotted under a mutex once per step, since gameplay may write
them while `OnStep` reads.

`JoltWaterVolumeComponent` (runtime) and `EditorJoltWaterVolumeComponent` (editor) follow
the same editor/runtime split as the physics gem, and both draw the volume as a
translucent box through the shared `DrawWaterVolume` — the visual is driven by the same
transform and dimensions the solver uses, so the two cannot drift apart.

`Include/JoltBuoyancy/JoltBuoyancyBus.h` exposes fluid density, linear and angular drag,
the current, enable/disable, and a submerged-body count for diagnosing a volume that
appears to be doing nothing.

## Building and testing

The test project ships `build-env.cmd`, which sets up MSVC/CMake/Ninja and then runs
whatever you pass it:

```
cmd /c "C:\Users\jorge\O3DE\Projects\JoltPhysicsTest\build-env.cmd cmake --build C:\Users\jorge\O3DE\Projects\JoltPhysicsTest\build\windows --target JoltBuoyancy JoltBuoyancy.Tests --config profile"
```

```
cd build\windows\bin\profile
.\AzTestRunner.exe JoltBuoyancy.Tests.dll AzRunUnitTests    # 8 tests
```

Check the process exit code, not the console text.

`Code/Tests/JoltWaterVolumeTests.cpp` builds a **plain Jolt world** rather than using the
physics gem's scene, so the gem is testable on its own — which also proves the API
dependency is all it needs. The eight: a light body floats, a dense one sinks, a denser
fluid floats a body that would otherwise sink, a body outside the volume falls freely,
disabling stops the effect and re-enabling resumes it, a current carries a body along,
and attaching to nothing fails.

## Test level

`JoltPhysicsTest/Levels/Buoyancy`, regenerated with `python gen_buoyancy_level.py`.
Three lanes, readable from the side:

- **Pool** (y=0) — water z=0→4. `Wood` 400 kg/m³ floats high, `Neutral` 1000 hovers,
  `Stone` 2500 sinks, and a **static** `Anchor` in the water must not move (buoyancy
  applies only to dynamic bodies).
- **River** (y=12) — the same water with a 3 m/s current carrying a `Raft`.
- **Dry** (y=−12) — no water, holding `DryControl`, identical to `Wood`.

The dry control is what makes the level meaningful: without buoyancy every pool body
would end up exactly where it does. Bodies are 1 m³ boxes, so mass *is* density against
1000 kg/m³ water, and they drop from z=7 onto a surface at z=4.

## Known limitations

- **Boxes only.** The volume is an oriented box; sphere and mesh water, and a real
  surface from Atom's water rendering, are unimplemented.
- **Overlapping volumes both apply impulses to the same body** — "double buoyancy". Not
  guarded against.
- **Buoyancy applies to dynamic bodies only**, by design (a static body in water is a
  pier, not a boat).

## Two traps worth keeping written down

1. **A semicolon anywhere in a `gem.json` string value breaks the build.**
   `C:\O3DE\26.05\cmake\O3DEJson.cmake:54` passes the file contents *unquoted* to
   `string(JSON …)`, and CMake splits unquoted variables on `;`, truncating the
   document. The error points at a valid line and says "value, object or array
   expected".
2. **Jolt's default gravity is −Y; O3DE is Z up.** The physics gem sets Z-down from the
   scene config, so this only bites when creating a raw `JPH::PhysicsSystem` — as the
   tests do. Symptom: bodies appear frozen at their spawn height because they are
   falling sideways.

## Status

**Observed in the editor (2026-07-26):** bodies float. The pool lane behaves as the unit
tests predict — this is the confirmation the notes this file replaced were still waiting
on, and it closes the last gap between "the maths is right" and "the feature works".

**Also verified:** the gem and its editor module build; 8/8 unit tests pass against a
real Jolt world; the level prefab is valid with the right editor components and masses;
the game launcher loads the level and simulates it for 30 s with no crash, exercising
the `Activate` → `AddStepListener` path that used to crash.

## License

MIT, matching the JoltPhysics gem and Jolt itself.
