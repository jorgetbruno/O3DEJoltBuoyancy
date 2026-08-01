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
them while `OnStep` reads. A volume can be a **box or a sphere**; a sphere is sized by
its X dimension and its surface sits at the top, like a tank filled to the brim.

### Waves, and arbitrary surfaces

The surface is sampled **per body**, not once for the volume, which is what lets it be a
wave rather than a plane: two boats on one swell sit at different heights and tilt with
their own part of it. The wave is evaluated in the volume's local space, so it rides a
tilted volume rather than ignoring the tilt, and its normal comes from three
finite-difference taps. The phase advances from the step delta rather than a wall clock,
so a paused game has a still surface, and it wraps each wave period so a float32 does not
lose precision over a long session.

`SetSurfaceFunction` replaces the built-in surface entirely, for water that has to line
up with something the gem knows nothing about. The volume still decides *which* bodies
are considered, since that comes from its bounds - which are padded upward by the wave
amplitude so a body riding a crest does not fall out of the query.

### Who owns a body

Overlapping volumes used to both apply an impulse to the same body. Volumes now register
per physics system, and each works out independently whether it owns a given body: the
one holding it deepest below its own surface wins. Every volume reaches the same answer
from the same data, which matters because Jolt runs step listeners on several jobs at
once - claiming first-come-first-served would depend on job order and vary frame to
frame. A body whose centre sits in a neighbour is left to that neighbour even when it
still overlaps this volume's box, which is the straddling case. Ownership sticks unless a
neighbour holds the body meaningfully deeper (`OwnershipHysteresis`), so a body drifting
along a seam does not change hands every few steps.

### Sleeping bodies

A sleeping body ignores water: `ApplyBuoyancyImpulse` does not wake anything. `OnStep`
queues sleepers and `WakePendingBodies` activates them **after** the step, because waking
takes the body mutex the step is holding. Only bodies in water that has *changed* are
woken, tracked by a generation counter - waking every sleeper every step would keep every
floating body permanently awake. A body that falls asleep is still in the water, so it
stays in the submerged set and keeps being counted.

### Per-body control, and events

A **Jolt Buoyancy Override** component gives one entity an explicit buoyancy factor,
drag multipliers, or exclusion from water entirely. Explicit factor is the sealed-hull
case: a boat is mostly air, but its collider volume says otherwise, so the derived
density sinks it. Overrides go through a registry rather than an EBus, because volumes
read them from Jolt's job threads where dispatching to gameplay code is off limits.

`JoltWaterVolumeNotificationBus` reports bodies entering and leaving, with the entry
speed along the surface normal so a handler can tell a splash from a drift. Both are
raised after the step, never from inside it.

`JoltWaterVolumeComponent` (runtime) and `EditorJoltWaterVolumeComponent` (editor) follow
the same editor/runtime split as the physics gem, and both draw the volume as a
translucent box through the shared `DrawWaterVolume` — the visual is driven by the same
transform and dimensions the solver uses, so the two cannot drift apart.

`Include/JoltBuoyancy/JoltBuoyancyBus.h` exposes fluid density, linear and angular drag,
the current, dimensions, the whole settings block, every wave parameter, enable/disable,
a submerged-body count, per-body submerged fraction (opt-in, since Jolt computes it
internally but does not hand it back), and the gameplay queries `IsPointUnderwater`,
`GetSurfacePositionAt`, `GetSurfaceNormalAt` and `GetDepthAt`.

Everything is reflected to the **BehaviorContext**, so Lua and Script Canvas can change
water and hear about splashes. See *Scripting* below.

A volume attaches to the default scene unless a scene name is authored, and its enabled
state is serialized, so a level can hold water that starts switched off.

## Scripting

`Assets/Scripts/WaterSplash.lua` is a working example: attach it to a water volume entity
with the **Lua Script** component and it reacts to bodies entering and leaving.

```lua
function WaterSplash:OnActivate()
    self.waterHandler = JoltWaterVolumeNotificationBus.Connect(self, self.entityId)
end

function WaterSplash:OnBodyEnteredWater(bodyEntityId, speed)
    if speed >= self.Properties.MinimumSplashSpeed then
        -- spawn a splash
    end
end
```

The notification bus is addressed by the **water volume's** entity, so `self.entityId` is
the right address when the script sits on the volume. Both callbacks are raised after the
physics step, never from inside it, so a handler may spawn effects or touch the body it
was handed.

Requests work the same way:

```lua
JoltWaterVolumeRequestBus.Event.SetWavesEnabled(self.entityId, true)
JoltBuoyancyOverrideRequestBus.Event.SetBuoyancyMode(boatId, JoltBuoyancyMode_Explicit)
JoltBuoyancyOverrideRequestBus.Event.SetBuoyancyFactor(boatId, 2.0)
local wet = JoltWaterVolumeRequestBus.Event.IsPointUnderwater(self.entityId, somePoint)
```

One trap worth knowing: **`EntityId(1234)` does not work in Lua.** The numeric constructor
is not reflected, and it quietly yields an invalid id rather than failing, so a bus call
made with one silently goes nowhere. Get ids the normal way - `self.entityId`, a component
property, or something handed to you by a callback.

`JoltBuoyancyLuaTests` runs real Lua against the real behavior context: driving an
override, reading values back, and receiving a splash in a Lua handler. That is a
stronger check than `JoltBuoyancyScriptReflectionTests`, which only proves the buses are
*listed* - an event can be reflected under a name script cannot call, or take a type
script cannot construct, and still appear in the registry.

**Script Canvas is not covered.** The same reflection is what Script Canvas nodes are
generated from, so the buses should appear there, but nothing here has been opened in the
Script Canvas editor and no graph has been run.

## Building and testing

The test project ships `build-env.cmd`, which sets up MSVC/CMake/Ninja and then runs
whatever you pass it:

```
cmd /c "C:\Users\jorge\O3DE\Projects\JoltPhysicsTest\build-env.cmd cmake --build C:\Users\jorge\O3DE\Projects\JoltPhysicsTest\build\windows --target JoltBuoyancy JoltBuoyancy.Tests --config profile"
```

```
cd build\windows\bin\profile
.\AzTestRunner.exe JoltBuoyancy.Tests.dll AzRunUnitTests    # 44 tests
```

Check the process exit code, not the console text.

`Code/Tests/JoltWaterVolumeTests.cpp` builds a **plain Jolt world** rather than using the
physics gem's scene, so the gem is testable on its own — which also proves the API
dependency is all it needs. It covers flotation by density, tilted volumes, compound
shapes, sphere volumes, waves and custom surfaces, per-body overrides and drag
multipliers, enter/exit events, submerged fraction, sleeping bodies waking and staying
counted, and the overlap cases including a body straddling two adjacent volumes.

`JoltWaterVolumeComponentTests.cpp` covers the layer above: a setter that updated only
the component's own copy and never reached the volume would pass every physics test and
still do nothing in a level. `JoltBuoyancyScriptReflectionTests.cpp` pins the script
surface.

One gap worth naming: the **collision-group filter** cannot be exercised here, because
resolving a group to a mask needs the physics gem's bus, which the standalone Jolt world
has no handler for.

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

- **Boxes and spheres only.** Mesh-shaped water is unimplemented: a volume is a
  containment test plus a surface function, not a Jolt body, so an arbitrary mesh would
  need point-in-mesh queries this gem does not have. A real rendered surface from Atom's
  water rendering is also unimplemented — the drawing here is debug geometry.
- **Ownership between overlapping volumes is a hard switch** once the hysteresis margin
  is crossed. A body crossing from a river into a pool changes current and density in one
  step rather than blending across the boundary.
- **One wave train.** The built-in surface is a single sine; anything richer needs
  `SetSurfaceFunction`.
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

**Also verified:** the gem and its editor module build; 44/44 unit tests pass against a
real Jolt world; the level prefab is valid with the right editor components and masses;
the game launcher loads the level and simulates it for 30 s with no crash, exercising
the `Activate` → `AddStepListener` path that used to crash.

**Not observed in the editor:** the box component mode's drag handles, the edit-mode
preview (water simulating in the Edit viewport), the tessellated wave surface, and the
sea and hull lanes added to the test level. Those build and are covered by unit tests
wherever a unit test can reach them, but none has been looked at.

## License

MIT, matching the JoltPhysics gem and Jolt itself.
