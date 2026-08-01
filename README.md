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

### The sea

Waves are synthesised from a **spectrum**, not authored one at a time. You give it a
Beaufort force, a fetch, a wind direction and a spread; it produces a set of Gerstner
components. That way the physical relationships hold by construction - author wavelengths
and speeds by hand and it is easy to make a sea that reads as wrong without being able to
say why.

Speeds are never authored. Each component takes its frequency from the deep-water
dispersion relation, `ω² = gk`, so long swell outruns short chop. Give the spectrum a
water depth and it uses the shallow form, `ω² = gk·tanh(kd)`, where long waves feel the
bottom, slow and shorten - swell steepening as it runs into a beach.

Gerstner rather than plain sine: points are dragged horizontally toward the crests, which
sharpens peaks and broadens troughs. That is the visual signature of open water. It also
means **the surface is not a heightfield** - the water above a given XY is not the wave
evaluated at that XY, because the point that ends up there started somewhere else.
`SampleAbove` inverts that displacement before evaluating. Skip it and boats sit visibly
off the crests, leaning the wrong way on a steep face.

Each component keeps its own phase, wrapped separately. There is no shared clock: with
several incommensurate periods there is nothing to wrap one to, so it would either lose
float32 precision over a long session or jump the surface at every wrap.

The surface is sampled at **several points across each body** and a plane fitted through
them. One sample at the centre of mass gives a hull the water under its middle and nothing
else, so a boat long enough to straddle a crest rises and falls without pitching. Bodies
much smaller than the shortest wave skip this - they cannot straddle anything.

Water has **orbital velocity**: it moves with the wave at a crest and against it in a
trough. That is what makes a boat surge down the face of a swell and flotsam gather in
lines. It is computed analytically alongside the displacement and added to the volume's
own current per body.

`SetSurfaceFunction` replaces the built-in surface entirely, for water that has to line up
with something the gem knows nothing about. The volume still decides *which* bodies are
considered, since that comes from its bounds - which are padded by the summed wave
amplitude and the horizontal displacement, so a body riding a crest does not fall out of
the broadphase query. One contract on it: **the returned position's XY must equal the query
point's XY** - only height and normal may differ. The footprint fit subtracts each tap's own
offset back out, so a surface that answers about a different column smears its samples
sideways. The built-in one honours this by inverting the displacement and then reporting at
the column it was asked about.

**Why a spectrum and not a component list.** An FFT ocean consumes the same thing:
Tessendorf synthesis runs over a Phillips or JONSWAP spectrum. Swapping the Gerstner sum
for an FFT later replaces the synthesis and leaves the authored data, the editor UI, the
bus and the tests standing. A hand-authored component array would have had to be thrown
away. A Gerstner sum tops out around 8-16 components and gets you *convincing*; FFT gets
you *photoreal* and moves the physics onto a readback texture or a low-res CPU FFT, routed
through `SetSurfaceFunction`.

### Hydrodynamics

Jolt does more here than it is usually given credit for. Its drag is **already quadratic** -
its own comment says "instead of eq 2.5.14 we use a quadratic drag formula" - and it is
**already directional**, projecting the body's local bounding box along the flow, so a long
hull presents less area end-on than broadside.

What a bounding box cannot express is a hull being far more streamlined than its box, which
is most hulls. The override carries a per-axis scale that refines Jolt's figure: Jolt is
handed the isotropic floor of the three axes, and the remainder is applied in the same
quadratic, area-projected form. That split is approximate - each half clamps independently,
and Jolt applies at the centre of buoyancy while the remainder applies at the centre of
mass - so setting all three axes alike does not exactly reproduce an unsplit drag. The
anisotropy is the point.

Two things Jolt gets wrong for a surface vessel, and both are corrected here. It scales its
**angular** drag by the submerged fraction but takes the **linear** drag area from the whole
shape's bounding box, so a hull floating with a tenth of itself wet dragged as though fully
immersed - superstructure included. And it infers the density of the water from the buoyancy
factor, `fluid_density = buoyancy / (totalVolume * inverseMass)`, which is exactly right in
Automatic mode (the factor is a density ratio, so the body density multiplies back out) and
wrong under an Explicit one, where a sealed hull asking for a factor of 3 to float correctly
paid three times the drag for it. The volume works out the submerged volume itself and hands
it to Jolt's volume-taking `ApplyBuoyancyImpulse` overload rather than letting Jolt compute
it and keep it, so both corrections cost nothing - it is the same single walk of the shape
either way, and reading the submerged fraction back is now free too.

An explicit factor therefore buys buoyancy and only buoyancy. It also makes such a hull a
far more lightly damped float than it was, which is the correct physics and worth knowing
before retuning anything.

**Added mass** is genuinely missing from Jolt and is approximated. Doing it properly means
adding to the solver's mass matrix, which Jolt does not expose, so this resists the change
in velocity after the fact. It damps acceleration rather than making the body heavier. That
is the honest description and it is labelled as such everywhere it appears.

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

### Extent

A volume is a **box**, a **sphere**, or a **plane** - everything below the surface within a
horizontal extent, with no floor at the authored Z dimension. An ocean is not a box: a body
that sinks out of the bottom of one abruptly weighs its full dry weight again.

"No floor" still needs a number, because **the broadphase is queried with a finite box and
that query is the only thing deciding which bodies are looked at at all**. `MaxDepth` is
where the bottom goes, defaulting to 10 km - deeper than any playable world. `Contains`
answers with the same number, deliberately: the two used to disagree, so a plane silently
stopped applying buoyancy one Z dimension below its surface while `IsPointUnderwater` went
on saying the body was in the water. That is the worst shape a bug can take, and the reason
the two are now computed from one figure rather than described the same way in two places.

Nor can an ocean be one enormous box horizontally. A volume can **follow an entity**,
recentring horizontally each frame so the queried region stays small while the water reads
as unbounded. Horizontal only - moving the surface with the camera would make the sea rise
and fall as the player travels.

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
.\AzTestRunner.exe JoltBuoyancy.Tests.dll AzRunUnitTests    # 76 tests
```

Check the process exit code, not the console text.

`Code/Tests/JoltWaterVolumeTests.cpp` builds a **plain Jolt world** rather than using the
physics gem's scene, so the gem is testable on its own — which also proves the API
dependency is all it needs. It covers flotation by density, tilted volumes, compound
shapes, sphere volumes, waves and custom surfaces, per-body overrides and drag
multipliers, enter/exit events, submerged fraction, sleeping bodies waking and staying
counted, the drag corrections above (a tenth-submerged body taking about a tenth of the
drag, and an explicit factor changing lift without changing drag), and the overlap cases
including a body straddling two adjacent volumes.

`JoltGerstnerWaveTests.cpp` ends with the CPU/GPU parity fixture. Those cases evaluate wave
sets **built by hand** — `SetComponents` bypasses synthesis — against values derived from
the formula in the comment above each assertion, not recorded from this implementation.
Recorded numbers catch later drift but cannot catch an error that was already there when
they were written down, and they give a shader author nothing to diff against. One case
runs a wave diagonally on purpose: everything axis-aligned has a zero cross-derivative, so
a shader that wrote `d.x * d.x` where it meant `d.x * d.y` would pass all the rest.

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
- **No rendered water surface.** `Assets/Shaders/GerstnerWaves.azsli` implements the same
  wave function for a vertex shader and documents the parity contract, but **nothing is
  wired into Atom** - no material, no render feature, no mesh. The drawing you see is debug
  geometry. This is the largest outstanding piece of work.
- **Shoreline is one depth per volume.** Water depth shortens the long waves, which is the
  physics behind shoaling, but a real shoreline needs a depth that varies with the sea
  floor - terrain access the gem does not have and should not take on speculatively.
- **A Gerstner sum, not an FFT.** Convincing rather than photoreal; see above for what
  switching would cost.
- **Added mass is an approximation**, as described above.
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

**Also verified:** the gem and its editor module build; 76/76 unit tests pass against a
real Jolt world; the level prefab is valid with the right editor components and masses;
the game launcher loads the level and simulates it for 30 s with no crash, exercising
the `Activate` → `AddStepListener` path that used to crash.

**Not observed in the editor:** the box component mode's drag handles, the edit-mode
preview, the tessellated wave surface, the sea and hull lanes in the test level, and every
part of the sea model added since. All of it builds and is covered by unit tests wherever a
unit test can reach, but none of it has been looked at.

**Not run at all:** the shader in `Assets/Shaders/`. It is written against the same spec as
the CPU evaluation and the parity contract is documented at the top of the file, but there
is no Atom material or render feature to run it, and no way to verify it here.

## License

MIT, matching the JoltPhysics gem and Jolt itself.
