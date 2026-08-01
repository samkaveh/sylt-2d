# Pinball Expansion Guide

This document describes how to extend the pinball game with new board elements,
physics behaviors, visual effects, and editor tools.

---

## Table of Contents

1. [Project Structure](#project-structure)
2. [Adding a New Board Element](#adding-a-new-board-element)
3. [Physics Behaviors](#physics-behaviors)
4. [Metaball & Liquid Effects](#metaball--liquid-effects)
5. [Joint-Based Mechanical Elements](#joint-based-mechanical-elements)
6. [Scoring & Game Logic](#scoring--game-logic)
7. [Visual Effects & Rendering](#visual-effects--rendering)
8. [Editor Tooling](#editor-tooling)
9. [Board Serialization](#board-serialization)
10. [Testing & Debugging](#testing--debugging)

---

## Project Structure

```
examples/pinball/
  Cargo.toml          # Package manifest (nannou, nannou_egui, sylt-2d)
  boards/             # Saved board JSON files
  EXPANSION_GUIDE.md  # This file
  src/
    main.rs           # Application entry point, all game logic
```

The entire application is a single `main.rs` file following the same pattern as
`examples/samples/src/main.rs`. It uses:

- **sylt-2d** for rigid body physics (`Body`, `Joint`, `World`)
- **nannou** for windowing, input, and 2D rendering
- **nannou_egui** for the editor and HUD panels

---

## Adding a New Board Element

Every placeable element is defined in the `ElementKind` enum. To add a new one:

### Step 1: Define the variant

```rust
// In the ElementKind enum:
ElementKind::MyNewElement {
    param_a: f32,
    param_b: u32,
},
```

### Step 2: Add editor tool (optional)

```rust
// In the EditTool enum:
EditTool::MyNewElement,

// In EditTool::name():
EditTool::MyNewElement => "My New Element",
```

### Step 3: Add editor properties

Add fields to `EditorState` for the configurable parameters, and add a match
arm in the editor UI section of `update()` to expose sliders.

### Step 4: Add physics body creation

In `build_element_bodies()`, create the physics bodies for the new element:

```rust
ElementKind::MyNewElement { param_a, param_b } => {
    let mut body = Body::new(Vec2::new(1.0, 1.0), f32::MAX);
    body.position = elem.position;
    body.rotation = elem.rotation;
    body.friction = 0.3;
    model.world.add_body(body);
}
```

### Step 5: Add rendering

In `view()`, add rendering logic for the new element type. Use `draw.rect()`,
`draw.ellipse()`, `draw.polygon()`, or `draw.line()` with appropriate colors.

### Step 6: Add click placement

In `mouse_pressed()`, add a match arm for the new tool to handle placement:

```rust
EditTool::MyNewElement => {
    let kind = ElementKind::MyNewElement {
        param_a: editor.param_a,
        param_b: editor.param_b,
    };
    let id = model.next_id;
    model.next_id += 1;
    model.elements.push(BoardElement {
        id,
        position: pos,
        rotation: 0.0,
        kind,
        color: colors[color_idx % colors.len()],
    });
}
```

---

## Physics Behaviors

### Applying Forces

Use `body.add_force(force)` during `update()` before `world.step()`:

```rust
fn apply_custom_physics(model: &mut Model) {
    for body in model.world.bodies.iter() {
        if body.borrow().inv_mass == 0.0 { continue; }
        // Example: attract to center
        let pos = body.borrow().position;
        let center = Vec2::new(0.0, 10.0);
        let dir = center - pos;
        let force = dir * 0.5;
        body.borrow_mut().add_force(force);
    }
}
```

### Angular Velocity Control

For flippers and moving obstacles, set `body.angular_velocity` directly:

```rust
body.angular_velocity = target_angular_speed;
```

### Custom Collision Response

Check `world.arbiters` after each step to detect collisions and apply
custom effects:

```rust
for (_, arbiter) in model.world.arbiters.iter() {
    for contact in arbiter.contacts.iter() {
        if let Some(c) = contact {
            let body1_id = arbiter.body1.borrow().id;
            let body2_id = arbiter.body2.borrow().id;
            // Apply custom logic based on body IDs
        }
    }
}
```

---

## Metaball & Liquid Effects

The game uses `sylt_2d::metaball` for rendering fluid pools. Each fluid pool is
a static circle body whose ID is tracked in `play.metaball_bodies`.

### How Fluid Rendering Works

1. Collect all metaball body positions into `Vec<Metaball>`
2. Cluster them with `cluster_metaballs()`
3. For each cluster, compute marching squares contours with
   `marching_squares_debug()`
4. Draw the resulting polygons with translucent fill

### Adding New Fluid Types

1. Add a variant to `FluidType`:

```rust
impl FluidType {
    fn name(&self) -> &str {
        match self {
            FluidType::Water => "Water",
            FluidType::Slime => "Slime",
            FluidType::Lava => "Lava",
            FluidType::Acid => "Acid",
            FluidType::Mercury => "Mercury", // new
        }
    }
}
```

2. Add drag coefficient in `apply_fluid_drag()`:

```rust
FluidType::Mercury => *viscosity * 2.0,
```

3. Add color in the editor rendering:

```rust
FluidType::Mercury => rgba(0.7, 0.7, 0.8, 0.5),
```

### Tuning Metaball Parameters

| Parameter | Effect | Typical Range |
|-----------|--------|---------------|
| `radius` | Size of the fluid pool | 0.5 - 5.0 |
| `viscosity` | Drag strength on ball | 0.1 - 3.0 |
| `metaball_threshold` | Contour smoothness | 0.3 - 1.0 |
| `metaball_resolution` | Detail level (grid cells) | 20 - 80 |

---

## Joint-Based Mechanical Elements

### Chain / Pendulum

Chains are built from multiple rectangular `Body` objects connected by `Joint`
instances. The first link is anchored to a static body at the top.

Key parameters:
- `link_count`: Number of chain links (2-20)
- `total_length`: Total chain length (1.0-10.0)
- `end_mass`: Mass of the bottom link (5.0-50.0)

The pendulum weight at the bottom creates swinging motion due to gravity.

### Soft Bridge / Trampoline

A series of plank bodies connected with soft joints (high `softness`, tuned
`bias_factor`). The spring-damper system is:

```
omega = 2 * PI * frequency_hz
d = 2 * mass * damping_ratio * omega
k = mass * omega * omega
softness = 1.0 / (d + dt * k)
bias_factor = dt * k / (d + dt * k)
```

### Building a Rotating Spinner

```rust
// Create spinner body (low friction, moderate mass)
let mut spinner = Body::new(Vec2::new(3.0, 0.3), 5.0);
spinner.position = center;
spinner.friction = 0.01;

// Create anchor
let mut anchor = Body::new(Vec2::new(0.1, 0.1), f32::MAX);
anchor.position = center;

// Create revolute joint
let joint = Joint::new(anchor, spinner, center, &model.world);
```

### Building a Teeter / Seesaw

```rust
// Plank
let mut plank = Body::new(Vec2::new(6.0, 0.3), 10.0);
plank.position = Vec2::new(0.0, 5.0);

// Center pivot anchor
let mut anchor = Body::new(Vec2::new(0.1, 0.1), f32::MAX);
anchor.position = Vec2::new(0.0, 5.0);

let joint = Joint::new(anchor, plank, Vec2::new(0.0, 5.0), &model.world);
```

---

## Scoring & Game Logic

### Score Detection

In `detect_scoring()`, after each physics step, iterate through arbiters to
find contacts. Check if the ball is near a scoring element:

```rust
ElementKind::Bumper { radius, boost, score } => {
    let dist = (elem.position - ball_pos).length();
    if dist < *radius + BALL_RADIUS + 0.5 {
        model.play.score += score;
        // Apply impulse
        let dir = vec2_normalize(body.position - elem.position);
        body.velocity = body.velocity + dir * *boost;
    }
}
```

### Adding Score Multipliers

Add a multiplier field to `PlayState` and multiply all score additions:

```rust
model.play.score += (score as f32 * model.play.multiplier) as u32;
```

### Adding Bonus Rounds

Track a timer in `PlayState` and activate bonus conditions:

```rust
if model.play.bonus_timer > 0.0 {
    model.play.multiplier = 2.0;
    model.play.bonus_timer -= model.time_step;
} else {
    model.play.multiplier = 1.0;
}
```

### Ball Life Management

- `balls_remaining` tracks remaining balls (default 3)
- When ball drains (y < -3.0), decrement and respawn
- When 0 remaining, set `game_over = true`
- Track `high_score` across plays

---

## Visual Effects & Rendering

### Color Coding

Current element colors are stored as `[f32; 3]` RGB in `BoardElement.color`.
Colors cycle through a predefined palette.

### Selection Highlight

Selected elements get a pulsing outline rendered in `draw_editor_elements()`.

### Bumper Glow

Bumpers render with two concentric ellipses: outer (base color) and inner
(brighter) for a glowing effect.

### Fluid Contours

Fluid pools render as translucent polygons from marching squares output. The
alpha channel controls transparency (0.4-0.6 typically).

### Adding Particle Effects

Add a `Vec<Particle>` to `Model` and update/render each frame:

```rust
struct Particle {
    position: Vec2,
    velocity: Vec2,
    lifetime: f32,
    color: [f32; 4],
}

// In update():
model.particles.retain_mut(|p| {
    p.position = p.position + p.velocity * model.time_step;
    p.lifetime -= model.time_step;
    p.lifetime > 0.0
});

// In view():
for p in &model.particles {
    draw.ellipse()
        .x_y(p.position.x, p.position.y)
        .radius(0.05)
        .color(rgba(p.color[0], p.color[1], p.color[2], p.color[3]));
}
```

---

## Editor Tooling

### Adding a New Tool

1. Add variant to `EditTool` enum
2. Add properties to `EditorState`
3. Add match arm in editor UI for property sliders
4. Add match arm in `mouse_pressed()` for placement
5. Add match arm in `draw_editor_elements()` for preview rendering

### Drag & Drop

The Select tool supports dragging elements. Key state:
- `editor.drag_start`: Position when drag began
- `editor.dragging`: Whether currently dragging
- `editor.selected_id`: ID of selected element

On `mouse_moved()`, update the selected element's position by the mouse delta.

### Hit Testing

Hit testing checks distance from click position to element:
- **Box**: Check x/y distance within half-width + padding
- **Circle/Bumper**: Check Euclidean distance within radius + padding
- **Other**: Use a 1.5-unit bounding box

---

## Board Serialization

### Current Data Model

```rust
struct BoardElement {
    id: usize,
    position: Vec2,
    rotation: f32,
    kind: ElementKind,
    color: [f32; 3],
}
```

### Saving a Board

To enable JSON saving, derive `Serialize` on all data structures:

```rust
#[derive(Serialize, Deserialize)]
struct BoardFile {
    name: String,
    version: u32,
    elements: Vec<BoardElementData>,
}
```

Then write with:

```rust
let json = serde_json::to_string_pretty(&board_file)?;
std::fs::write("boards/my_board.json", json)?;
```

### Loading a Board

```rust
let json = std::fs::read_to_string("boards/my_board.json")?;
let board_file: BoardFile = serde_json::from_str(&json)?;
```

### Pre-built Board Templates

Store in `boards/` directory:
- `boards/classic.json` - Traditional pinball layout
- `boards/bumper_madness.json` - Heavy bumper focus
- `boards/chain_chaos.json` - Pendulums and chains
- `boards/fluid_rush.json` - Liquid hazard heavy

---

## Testing & Debugging

### Physics Debug Visuals

Enable in the editor:
- **Show Contacts**: Renders contact points and normals from arbiters
- **Show Grid**: Renders a coordinate grid for element placement

### Keyboard Controls (Play Mode)

| Key | Action |
|-----|--------|
| A / Left Arrow | Activate left flipper |
| D / Right Arrow | Activate right flipper |
| Space (hold) | Charge plunger |
| Space (release) | Launch ball |
| R | Reset current play |
| TAB | Toggle Editor/Play mode |
| W/S/Q/E | Pan camera up/down/left/right |

### Common Issues

| Issue | Cause | Fix |
|-------|-------|-----|
| Ball falls through floor | Gravity too strong | Reduce gravity or increase iterations |
| Flippers don't respond | Missing anchor bodies | Ensure flipper pivot has an anchor body |
| Chain breaks apart | Joints too soft | Reduce `softness` on chain joints |
| Fluid doesn't render | Empty metaball list | Check metaball body IDs are tracked |
| Score doesn't increase | Wrong distance check | Verify scoring radius includes ball radius |

### Performance Tips

- Keep `world.iterations` at 100-200 for stable simulation
- Reduce `metaball_resolution` for complex fluid pools
- Limit total body count to under 200 for smooth 60fps
- Use `f32::MAX` mass for static (non-moving) bodies
- Avoid creating/destroying bodies every frame; reuse when possible
