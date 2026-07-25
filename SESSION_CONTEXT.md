# JoltBuoyancy — session handoff

Written to resume work in a fresh chat with no prior context. Everything below
was verified in the session that produced it unless it says otherwise.

## What this gem is

A separate O3DE gem that adds water to the Jolt physics backend **without the
JoltPhysics gem knowing anything about buoyancy**. A `Jolt Water Volume`
component turns an entity into a box of water; Jolt rigid bodies overlapping it
float, sink or drift according to their own density and the fluid settings.

It exists to prove the extension-gem pattern: features Jolt provides but the
physics gem does not wrap (buoyancy now, soft bodies later) can live in their
own gems.

## Where everything is

| Thing | Path |
|---|---|
| This gem | `C:\Users\jorge\O3DE\Gems\JoltBuoyancy` (own git repo, `f0e4a62`) |
| Physics gem it depends on | `C:\Users\jorge\O3DE\Gems\JoltPhysics` (own git repo, `1b9fd1b`) |
| Test project | `C:\Users\jorge\O3DE\Projects\JoltPhysicsTest` (own git repo, `61ddc6c`) |
| Test level | `JoltPhysicsTest\Levels\Buoyancy\Buoyancy.prefab` |
| Level generator | `JoltPhysicsTest\gen_buoyancy_level.py` |
| Engine | Installed SDK at `C:\O3DE\26.05` (Jolt pinned to v5.5.0) |

All three repos are **local only — nothing has been pushed**.

Both gems are registered with O3DE and enabled in JoltPhysicsTest.

## Build and test

The project ships `build-env.cmd`, which sets up MSVC/CMake/Ninja and then runs
whatever you pass it. Use it for every build command:

```
cmd /c "C:\Users\jorge\O3DE\Projects\JoltPhysicsTest\build-env.cmd cmake --build C:\Users\jorge\O3DE\Projects\JoltPhysicsTest\build\windows --target JoltBuoyancy JoltBuoyancy.Tests --config profile"
```

Run the tests from `build\windows\bin\profile`:

```
.\AzTestRunner.exe JoltBuoyancy.Tests.dll AzRunUnitTests     # 7 tests, all passing
.\AzTestRunner.exe JoltPhysics.Tests.dll AzRunUnitTests      # 128 tests, all passing
```

**Unity build gotcha (bites constantly):** editing a `.cpp` often relinks the
DLL *without recompiling it*, so you test a stale binary. Symptom: results are
byte-identical after a change that should alter them. Fix — touch the unity
files before building:

```powershell
$base = "C:\Users\jorge\O3DE\Projects\JoltPhysicsTest\build\windows\External\<GEM>-<hash>\Code\CMakeFiles"
Get-ChildItem "$base\<TARGET>.dir\Unity\*.cxx" | ForEach-Object { $_.LastWriteTime = Get-Date }
```

## How it works

`JoltWaterVolume` (`Source/Clients/JoltWaterVolume.h/.cpp`) is the core. Three
things about it are load-bearing:

1. **It is a `JPH::PhysicsStepListener`, not a tick handler.** The impulse has
   to land inside the step it belongs to, at the delta time the solver is about
   to integrate.
2. **Jolt calls step listeners with every body mutex already held.** Bodies are
   therefore reached through `GetBodyLockInterfaceNoLock()` and none are added
   or removed there. Using `BodyInterface` inside `OnStep` would deadlock.
3. **The buoyancy factor is derived per body** from its own density (mass ÷
   shape volume) against the fluid density, so a dense body sinks and a light
   one bobs. Jolt's factor is relative: 1 is neutral, above 1 floats.

The volume's **local +Z face is the water surface**, so a tilted volume gives a
tilted surface. Settings are snapshotted under a mutex once per step, since
gameplay may write them while `OnStep` reads.

`JoltWaterVolumeComponent` (runtime) + `EditorJoltWaterVolumeComponent` (editor,
draws the box and spawns the runtime one via `BuildGameEntity`) follow the same
editor/runtime split as the physics gem. `Include/JoltBuoyancy/JoltBuoyancyBus.h`
exposes density, drags, current, enable/disable and a submerged-body count.

## How it reaches Jolt

Two mechanisms, both deliberate:

- **`Gem::JoltPhysics.API`** is an INTERFACE target that re-exports `Jolt::Jolt`
  *with the compile definitions the physics gem built Jolt with*. Matching those
  exactly is what keeps the modules ABI compatible — a mismatch is an ODR
  violation that corrupts memory rather than failing to link. Do not link Jolt
  any other way.
- **`JoltPhysicsSystemRequestBus::GetNativePhysicsSystem(sceneHandle)`** was
  added to the physics gem's public API for this (`e4089e7`). It returns the
  `JPH::PhysicsSystem*` only for scenes that gem owns, and null otherwise, so
  this gem is inert rather than dangerous under a different physics backend.
  Casting a scene's native pointer directly would not be safe: `AzPhysics::Scene`
  exposes `GetNativePointer()` but no type identification.

`JoltWaterVolume::AttachToPhysicsSystem(JPH::PhysicsSystem*)` is a public seam
used both by `Attach(sceneHandle)` and by the tests.

## Tests

`Code/Tests/JoltWaterVolumeTests.cpp` builds a **plain Jolt world** rather than
using the physics gem's scene, so the gem is testable on its own — which also
proves the API dependency is all it needs. Seven tests: a light body floats, a
dense one sinks, a denser fluid floats a body that would otherwise sink, a body
outside the volume falls freely, disabling stops the effect and re-enabling
resumes it, a current carries a body along, and attaching to nothing fails.

## Two traps that cost real time

1. **A semicolon anywhere in a `gem.json` string value breaks the build.**
   `C:\O3DE\26.05\cmake\O3DEJson.cmake:54` passes the file contents *unquoted*
   to `string(JSON …)`, and CMake splits unquoted variables on `;`, truncating
   the document. The error points at a valid line and says "value, object or
   array expected". Keep semicolons out of gem.json.
2. **Jolt's default gravity is −Y; O3DE is Z up.** The physics gem sets Z-down
   from the scene config, so this only bites when creating a raw
   `JPH::PhysicsSystem` — as the tests do. Symptom: bodies appear frozen at
   their spawn height because they are falling sideways.

## The test level

`Levels/Buoyancy`, regenerate with `python gen_buoyancy_level.py`. Three lanes,
readable from the side:

- **Pool** (y=0) — water z=0→4. `Wood` 400 kg/m³ floats high, `Neutral` 1000
  hovers, `Stone` 2500 sinks, and a **static** `Anchor` in the water must not
  move (buoyancy applies only to dynamic bodies).
- **River** (y=12) — same water with a 3 m/s current carrying a `Raft`.
- **Dry** (y=−12) — no water, holding `DryControl`, identical to `Wood`.

The dry control is the comparison that makes the level meaningful: without
buoyancy every pool body would end up exactly where it does. Bodies are 1 m³
boxes, so mass *is* density against 1000 kg/m³ water. They drop from z=7 onto a
surface at z=4.

Authored with the **editor** components, so colliders and the water volume draw
in the viewport — unlike the older `SmokeBox` level, which predates the
editor/runtime split.

## Status: verified vs not

**Verified:** both gems and the editor modules compile; JoltBuoyancy 8/8 and
JoltPhysics 128/128 pass; the level prefab is valid JSON with all 8 entities,
the right editor component types and the masses written; both gems enabled in
the project. After the allocator fix, `JoltPhysicsTest.GameLauncher.exe
+LoadLevel levels/buoyancy/buoyancy.spawnable -rhi=null` loads the level,
activates its entities and simulates for 30 s with no crash and no dump — which
exercises the exact `Activate` → `AddStepListener` path that was crashing.

**Not verified:** the level has still not been opened in the **Editor** since
the fix (the launcher run above covers the runtime module; the editor module got
the identical fix but has not been run). And the buoyancy *behaviour* — what
actually floats where in this level — has never been observed anywhere; it is
predicted from unit tests against a real Jolt world.

## The Editor crash on entering game mode — found and fixed

Symptom: `Levels/SmokeBox` ran fine, `Levels/Buoyancy` crashed the moment you
enabled simulation or pressed play.

**Cause: each module gets its own copy of Jolt's allocation hooks, and this
gem never filled its own in.** Jolt allocates through five global function
pointers (`JPH::Allocate`, `Reallocate`, `Free`, `AlignedAllocate`,
`AlignedFree`). Jolt is statically linked into every module that uses it, so
each module owns its own copy and they start null. The physics gem installs
*its* copy in `JoltAllocator::Install`; that does nothing for this gem. Any Jolt
code the linker resolved locally — `PhysicsSystem::AddStepListener`, and the
header-only collision collectors in `OnStep` — then allocates through a null
pointer.

Entering game mode activated `JoltWaterVolumeComponent`, which called
`AddStepListener`, whose `mStepListeners.push_back` called
`JPH::Reallocate(nullptr, 0, 8)` — a jump to address 0.

Fix: `Source/Clients/JoltBuoyancyAllocator.{h,cpp}`, installed from the first
line of both `JoltBuoyancyModule` and `JoltBuoyancyEditorModule`. The hooks
forward to `AZ::SystemAllocator`, deliberately identical to the physics gem's,
because the allocations cross the module boundary: this module's
`AddStepListener` grows the array and the physics gem's `~PhysicsSystem` frees
it. `JPH::RegisterDefaultAllocator` would compile and run but routes to
malloc/free, handing the physics gem a block `AZ::SystemAllocator` never issued.

Why 7 passing tests missed it: the test module called
`JPH::RegisterDefaultAllocator()` in its own `SetUp`, so *its* copy of the
pointers was always valid. The tests now call `JoltBuoyancyAllocator::Install()`
instead, so there is one allocator story. Note the structural limit — a module
that forgets to call `Install` still crashes, and no unit test in another module
can catch that. Keep the call in every module entry point.

**Reading a crash dump without a debugger.** No `cdb`/WinDbg is installed and
O3DE's own `error.log` stack was useless (only the exception-filter frames), and
its "Attempt to write data to address 0x00000000" was wrong — the exception
parameters say `8`, an *execute* fault. `scratchpad/dmp2.py`-style minidump
parsing plus `dbghelp` via `ctypes` against `build\windows\bin\profile\*.pdb`
gave the real frames. Note the O3DE minidump's `CONTEXT` is at exception-stream
offset **160**, not 168.

## Reasonable next steps

- Open the level in the Editor and confirm the behaviour (the real gap).
- Any *future* extension gem that links Jolt hits the same allocator trap on its
  first Jolt call. The durable fix would be for the physics gem to export its
  installer from `JoltPhysics.API` so extension gems call one shared function
  instead of each copying `JoltAllocator`; this gem duplicates it today.
- Write an in-editor check for it. `JoltPhysicsTest\smoke_test.py` is the
  existing harness pattern for that.
- The volume is an oriented box only. Sphere/mesh water and a proper surface
  from Atom's water rendering are unimplemented.
- Overlapping volumes both apply impulses to the same body ("double buoyancy").
  Not guarded, documented only here.
- Soft bodies are **done, and they live inside the JoltPhysics gem**, not in an
  extension gem. The "body-adoption API" this section used to call a blocker was
  not one: `BodyInterface::CreateAndAddSoftBody` exists, so a soft body is an
  ordinary Jolt body and collides with rigid bodies today. Scene handles and
  collision events are still missing, which is exactly why it belongs inside the
  gem — see `Gems/JoltSoftBody/MIGRATED.md` for the reasoning and the file map.
  Test level: `Levels/SoftBody`, from `gen_softbody_level.py`.
- The dividing line that came out of it: a feature that **owns bodies** belongs
  in the physics gem, one that only **perturbs bodies someone else owns** can be
  an extension gem. Buoyancy is the second kind and correctly stays separate.
- **Hair does not exist in Jolt 5.5.0** — that still needs a Jolt upgrade first.
- The physics gem logs a pile of `duplicated Uuid` asserts at startup
  (`EditorJoltColliderComponentBase`, most joint configurations). Pre-existing,
  not investigated, and harmless as far as anything here shows — but they make a
  real assert easy to miss in the noise.
