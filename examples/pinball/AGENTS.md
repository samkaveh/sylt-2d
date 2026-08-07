# AGENTS.md

Development context for the **pinball** example project.

## What this is

A full-featured, arcade-style pinball game that also doubles as the most complex
demo of the `sylt-2d` physics engine. It exercises rigid bodies, joints,
chains, soft bridges, metaball fluid pools, sensors, scoring, combo systems,
a board editor, and a self-running demo tour.

Stack: Rust 2021, `nannou` 0.19 (window/rendering), `nannou_egui` 0.19 (editors),
`sylt-2d` (local physics crate at repo root `../..`, with the `log` feature).

## Running

```bash
# Normal interactive mode (editor first, then TAB into play)
cargo run --release

# Self-driving feature tour that builds a playfield, plays it, then
# walks through every editor tool. Screenshot whenever a bug appears.
cargo run --release -- --demo
```

Build/verify (both must pass clean):
```bash
cargo build --release
```

The only warnings emitted come from the `sylt-2d` lib crate
(`mismatched_lifetime_syntaxes` on `iter_bodies`) — these are pre-existing and
**not** caused by this project. The pinball crate itself should compile with no
warnings.

## Architecture (module map)

The refactored source lives under `src/`. `main.rs` used to be a ~3,3k-line
monolith; it is now a thin entrypoint that wires the modules together.

| Module      | Responsibility                                                              |
|-------------|----------------------------------------------------------------------------|
| `main.rs`   | `main()`, `model()` construction, `update()` frame loop, mode wiring.      |
| `state.rs`  | All `const`s, enums, and structs, including the central `Model`.           |
| `util.rs`   | Pure math + hit-testing helpers (no `Model` mutating).                     |
| `board.rs`  | Building the physical world: ball, plunger, fluid pools, cabinet walls,    |
|             | default/demo boards, and `enter_play_mode` / `enter_editor_mode`.          |
| `physics.rs`| Simulation behavior: fluid drag, fluid-particle animation, scoring,        |
|             | flipper servo drive, plunger charge/launch.                                |
| `demo.rs`   | The auto-playing feature tour (`update_demo`) + tool showcase helpers.     |
| `controls.rs`| Input handlers: raw window events, mouse, keyboard.                        |
| `ui.rs`     | `nannou_egui` panels: editor tool panel + play HUD. Returns action structs.|
| `render.rs` | `view()` and all drawing: world bodies, editor gizmos, labels, overlays.   |

## Key types & the `Model`

- `Model` (`state.rs`) is the single app state handed through all of nannou.
  Every subsystem reads/writes it via `&mut Model`.
- `GameMode::{Editor, Play}` — editor places `BoardElement`s; play builds a
  `World` from them.
- `ElementKind` — the union of placeable entities: `Wall`, `Bumper`, `Flipper`,
  `Chain`, `FluidPool`, `SoftBridge`, `Target`, `Drain`, `BallSpawn`.
- Note: the engine's own `Vec2` (`sylt_2d::math_utils::Vec2`) is preferred over
  nannou's `geom::Vec2` for physics values. Both appear in the code — keep them
  consistent per function.

## Important tuning constants (tweaking these changes feel/balance)

Defined in `state.rs`:

| Constant               | Value | Effect                                       |
|------------------------|-------|----------------------------------------------|
| `GRID_SNAP`            | 0.25  | Default editor snap spacing.                  |
| `BALL_RADIUS`          | 0.35  | Pinball size.                                 |
| `FLIPPER_UP_DELTA`     | 0.9   | Radiant rotation a flipper flips through.     |
| `PLUNGER_MAX_CHARGE`   | 55.0  | Max plunger launch speed. |
| `BALL_MAX_SPEED`       | 60.0  | Speed cap so scoring/serving stays sane.      |
| `BUMPER_COOLDOWN`      | 0.3   | Re-hit delay (s). |
| `TARGET_COOLDOWN`      | 0.5   | Re-hit delay for targets (s). |
| `COMBO_WINDOW`         | 3.0   | Combo reset window (s). |
| `BALL_SAVE_DURATION`   | 2.0   | Ball-save grace window (s). |
| `BALL_TRAIL_LENGTH`    | 12    | Trail ghost count. |

Gravity is set when building the `World` in `main()` (`World::new(..., -15.0)`),
and the plunger must launch in lane only (`x > 5.0 && y < 3.0`), not globally.

## Common pitfalls / gotchas (regressions we hit)

1. **Labels must be rasterised in screen space in one pass.** Never call
   `draw.to_frame()` on every label, and never mutate the borrow that `view()`
   already used. Collect text into `Vec<render::Label>` and present it with a
   **fresh** `app.draw()` **after** the world's `draw.to_frame(...)` call.
   Doing otherwise yields a black window or blurry text at non-default zoom.
2. **Ball must only launch from the plunger lane.** No global plunger fallback
   — the check in `physics::launch_plunger` guards `x > 5.0 && y < 3.0`.
3. **Demo board layout matters.** Ensure walls/guides don't block the demo
   ball's path to the bumpers/targets/pools. The `--demo` path is a first-class
   bug detector — run it after any physics/board change.
4. **Flippers pivot off-center.** `drive_flipper` drives both angular velocity
   *and* orbital linear velocity about the pivot; changing the pivot math
   without the joint will cause jitter.

## How to add a feature (workflow)

1. Read `EXPANSION_GUIDE.md` (design doc) and `IMPROVEMENT_PLAN.md`
   (prioritized feature backlog).
2. Add new `ElementKind` variants / state in `state.rs`.
3. Handle building/deploying the body in `board.rs` (the `build_element_bodies`
   match), physics in `physics.rs`, editor UX in `controls.rs`+`ui.rs`, and the
   visual in `render.rs` (both `draw_element_shape` and the `view()` body match).
4. Build with `cargo build --release`.
5. Run `cargo run --release` and `cargo run --release -- --demo`, confirming a
   well-performing, non-blank window with crisp text and lane-only ball launch.