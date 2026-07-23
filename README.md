# sylt-2d

A 2D physics engine built on [Box2D-lite](https://github.com/erincatto/box2d-lite) in Rust, with added convex polygon support, circle bodies, metaball rendering, sweep-and-prune broadphase, and structured error handling.

```toml
[dependencies]
sylt-2d = "0.1.0"

# Optional: enable JSON simulation logging
sylt-2d = { version = "0.1.0", features = ["log"] }
```

## Quick Start

```rust
use sylt_2d::world::World;
use sylt_2d::body::Body;
use sylt_2d::math_utils::Vec2;

fn main() {
    // Create a world with downward gravity, 80 solver iterations
    let mut world = World::new(Vec2::new(0.0, -10.0), 80);

    // Static ground (mass = f32::MAX)
    let mut ground = Body::new(Vec2::new(100.0, 1.0), f32::MAX);
    ground.position = Vec2::new(0.0, -10.0);
    world.add_body(ground);

    // Dynamic box
    let mut box_body = Body::new(Vec2::new(1.0, 1.0), 50.0);
    box_body.position = Vec2::new(0.0, 5.0);
    world.add_body(box_body);

    // Simulate
    for _ in 0..600 {
        world.step(1.0 / 60.0).unwrap();
    }
}
```

## Core Concepts

### World

The `World` manages all bodies, joints, and collision detection.

```rust
use sylt_2d::world::{World, WorldContext};

let mut world = World::new(
    Vec2::new(0.0, -10.0),  // gravity
    80,                      // solver iterations
);

// Add bodies
world.add_body(body);

// Add joints
world.add_joint(joint);

// Step the simulation
world.step(1.0 / 60.0)?;

// Iterate over bodies
for body in world.iter_bodies() {
    println!("{}: ({}, {})", body.id, body.position.x, body.position.y);
}

// Clear everything
world.clear();
```

**WorldContext** controls solver behavior (accessible via `world.world_context`):

| Field | Default | Description |
|-------|---------|-------------|
| `accumulate_impulse` | `true` | Accumulate impulses across frames for stability |
| `warm_starting` | `false` | Re-use previous frame's impulses to converge faster |
| `position_correction` | `true` | Push overlapping bodies apart to reduce sinking |

### Bodies

Three body shapes are supported:

#### Box

```rust
let mut body = Body::new(
    Vec2::new(2.0, 1.0),  // width (x) and height (y)
    50.0,                   // mass (f32::MAX for static)
);
body.position = Vec2::new(0.0, 5.0);
body.friction = 0.5;
world.add_body(body);
```

#### Convex Polygon

Vertices must form a convex shape. They are automatically oriented counterclockwise.

```rust
use sylt_2d::math_utils::Vec2;

let vertices = vec![
    Vec2::new(0.0, 1.0),   // top
    Vec2::new(-0.95, 0.31), // upper left
    Vec2::new(-0.59, -0.81),// lower left
    Vec2::new(0.59, -0.81), // lower right
    Vec2::new(0.95, 0.31),  // upper right
];
let mut pentagon = Body::new_polygon(vertices, 50.0);
pentagon.position = Vec2::new(-3.0, 5.0);
world.add_body(pentagon);
```

#### Circle

```rust
let mut ball = Body::new_circle(
    0.5,    // radius
    10.0,   // mass
);
ball.position = Vec2::new(2.0, 8.0);
world.add_body(ball);
```

#### Static Bodies

Pass `f32::MAX` as mass to create a body with infinite mass (cannot be moved by forces or collisions):

```rust
let ground = Body::new(Vec2::new(100.0, 1.0), f32::MAX);
```

#### Applying Forces

```rust
body.add_force(Vec2::new(100.0, 0.0)); // apply continuous force
```

#### Body Properties

| Field | Type | Description |
|-------|------|-------------|
| `position` | `Vec2` | World position |
| `rotation` | `f32` | Angle in radians |
| `velocity` | `Vec2` | Linear velocity |
| `angular_velocity` | `f32` | Angular velocity |
| `force` | `Vec2` | Accumulated force (cleared each step) |
| `torque` | `f32` | Accumulated torque (cleared each step) |
| `mass` | `f32` | Mass (f32::MAX = static) |
| `inv_mass` | `f32` | Inverse mass (0.0 = static) |
| `friction` | `f32` | Surface friction coefficient |
| `shape` | `Shape` | `Box`, `ConvexPolygon`, or `Circle` |
| `radius` | `f32` | Circle radius (0 for non-circles) |
| `moi` | `f32` | Moment of inertia |
| `id` | `usize` | Unique auto-incrementing ID |

### Joints

Joints connect two bodies at an anchor point, acting like a rigid rod (with optional softness for spring-like behavior).

```rust
use sylt_2d::joint::Joint;

let joint = Joint::new(
    body_a,          // first body
    body_b,          // second body
    anchor,          // world-space anchor point
    &world,          // needed to look up bodies by ID
);

// Spring-like joint (softness > 0)
let mut spring_joint = Joint::new(body_a, body_b, anchor, &world);
spring_joint.softness = 0.5;   // higher = more springy
spring_joint.bias_factor = 1.0;

world.add_joint(joint);
```

| Property | Default | Description |
|----------|---------|-------------|
| `softness` | `0.0` | 0 = rigid, >0 = spring-like elasticity |
| `bias_factor` | `0.2` | Position correction strength |
| `local_anchor_1` | computed | Anchor point in body 1's local frame |
| `local_anchor_2` | computed | Anchor point in body 2's local frame |

### Collision Detection

The engine uses a **sweep-and-prune (SAP) broadphase** for efficient pair finding, followed by narrow-phase collision detection based on shape pairs:

| Body A | Body B | Algorithm |
|--------|--------|-----------|
| Box | Box | SAT + Sutherland-Hodgman clipping |
| Circle | Circle | Distance check |
| Circle | Box/Polygon | Closest-point on polygon + point-in-polygon test |
| Box/Polygon | Circle | Same as above, normal flipped |
| Polygon | Polygon | SAT + polygon clipping |

Collision results are stored as `Arbiter` objects in `world.arbiters`, keyed by `ArbiterKey`. Each arbiter contains `Contact` points with position, normal, and impulse data.

## Metaballs

sylt-2d includes a metaball system that renders organic, blob-like shapes using marching squares. Metaballs are separate from the physics simulation — they are a rendering primitive that can follow physics bodies.

### Creating Metaballs

```rust
use sylt_2d::metaball::Metaball;
use sylt_2d::math_utils::Vec2;

let metaball = Metaball::new(
    Vec2::new(0.0, 0.0),  // position
    2.0,                    // radius
    1.0,                    // strength
);
```

The field value at any point is: `strength * radius^2 / distance^2`

### Clustering

When metaballs are far apart, they should render as separate blobs. The `cluster_metaballs` function groups nearby metaballs using union-find, based on whether the field at their midpoint exceeds the threshold:

```rust
use sylt_2d::metaball::cluster_metaballs;

let clusters = cluster_metaballs(&metaballs, threshold);
for cluster in &clusters {
    // Each cluster has its own metaballs and bounding box
    println!("Cluster with {} metaballs", cluster.metaballs.len());
}
```

### Computing Bounds

Metaball fields extend to infinity, so the marching squares grid needs bounds that fully contain the iso-surface:

```rust
use sylt_2d::metaball::compute_metaball_bounds;

let (bounds_min, bounds_max) = compute_metaball_bounds(
    &metaballs,
    0.5,    // threshold
    3.0,    // initial padding
);
```

### Marching Squares

Generate iso-surface polygons from the metaball field:

```rust
use sylt_2d::metaball::{marching_squares, marching_squares_debug};

// Simple: returns closed polygons
let polygons: Vec<Vec<(f32, f32)>> = marching_squares(
    &metaballs,
    bounds_min,
    bounds_max,
    50,     // resolution (grid cells per axis)
    0.5,    // threshold
);

// Debug: returns full info including grid, segments, open chains
let debug = marching_squares_debug(
    &metaballs,
    bounds_min,
    bounds_max,
    50,
    0.5,
);
// debug.polygons     - closed polygon contours
// debug.open_chains  - unclosed contour segments
// debug.segments     - raw line segments
// debug.grid         - sampled field values
// debug.grid_min/max - field value range
```

### Metaball Rendering Pattern

A typical rendering loop with metaballs attached to physics bodies:

```rust
use sylt_2d::metaball::{Metaball, cluster_metaballs, compute_metaball_bounds, marching_squares_debug};

// Collect metaballs from physics bodies
let metaballs: Vec<Metaball> = metaball_bodies
    .iter()
    .map(|&body_idx| {
        let b = world.bodies[body_idx].borrow();
        Metaball::new(
            Vec2::new(b.position.x, b.position.y),
            b.radius * 2.0,
            strength,
        )
    })
    .collect();

if !metaballs.is_empty() {
    let clusters = cluster_metaballs(&metaballs, cluster_threshold);

    for cluster in &clusters {
        let (min, max) = compute_metaball_bounds(&cluster.metaballs, threshold, 3.0);
        let debug = marching_squares_debug(&cluster.metaballs, min, max, resolution, threshold);

        // Render filled polygons
        for poly in &debug.polygons {
            if poly.len() >= 3 {
                // draw polygon...
            }
        }
    }
}
```

### Metaball Parameters

| Parameter | Typical Range | Description |
|-----------|---------------|-------------|
| `radius` | `0.5 - 3.0` | Field influence radius |
| `strength` | `0.5 - 5.0` | Field intensity multiplier |
| `threshold` | `0.1 - 2.0` | Iso-surface level (lower = bigger blob) |
| `resolution` | `20 - 100` | Grid cells per axis (higher = smoother) |
| `cluster_threshold` | `0.0 - 5.0` | Max midpoint field to merge clusters |

## ASCII Debug Rendering

The `draw` module provides terminal-based visualization using ANSI colors:

```rust
use sylt_2d::draw::*;
use sylt_2d::math_utils::Vec2;

let mut grid = make_grid(40);

// Draw shapes
add_box(&mut grid, body.position, body.width, body.rotation, 'X', style);
add_point(&mut grid, contact.position, 'C', red_style);
add_line(&mut grid, start, end, '*', style);

// Draw collision contacts
draw_collision_result(&mut grid, &contacts);

// Print to terminal
draw_grid(&mut grid);
```

### Color Styles

```rust
use sylt_2d::draw::{ColorStyle, TextColor, TextStyle};

let style = ColorStyle {
    text_color: TextColor::Red,
    background_color: Some(TextColor::Black),
    style: Some(TextStyle::Bold),
};

let styles = get_styles(); // 8 pre-defined styles
```

## JSON Logging

Enable the `log` feature to write per-step simulation state to a JSONL file:

```rust
use sylt_2d::log::Logger;

let mut logger = Logger::new("simulation.jsonl");
world.set_logger(logger);

// Each world.step() writes a JSON line with:
// - step number, dt, gravity
// - all body positions, velocities, rotations
// - all contact points and impulses
// - all joint states
```

Output format (one JSON object per line):

```json
{
  "step": 0,
  "dt": 0.016666,
  "gravity": {"x": 0.0, "y": -10.0},
  "bodies": [
    {"id": 0, "shape": "Box", "position": {"x": 0.0, "y": 5.0}, ...}
  ],
  "arbiters": [...],
  "joints": [...]
}
```

## Examples

### Running the Samples

```bash
cd examples/samples
cargo run --release
```

### Demo List

| # | Name | What it demonstrates |
|---|------|---------------------|
| 1 | Simple Shapes Falling | Static ground + dynamic box, pentagon, hexagon |
| 2 | Simple Pendulum | Joint connecting a box to a fixed anchor |
| 3 | Varying Friction | 5 boxes with different friction on a tilted ramp |
| 4 | Randomized Stacking | 10 stacked boxes with random offsets |
| 5 | Pyramid Stacking | 12-row pyramid (78 boxes) |
| 6 | A Teeter | Balanced plank with heavy object drop |
| 7 | Suspension Bridge | 15 spring-jointed planks forming a bridge |
| 8 | Dominos | Chain reaction with dominoes, ramps, and pendulums |
| 9 | Multi-pendulum | 15 spring-connected pendulum segments |
| 10 | Pawn and Pendulum | Compound polygon shapes (head + trunk) |
| 11 | Metaballs | 6 circles rendered as metaball iso-surfaces |
| 12 | 2 Metaballs (Static) | Static metaball contour rendering |
| 13 | 2 Metaballs (Physics) | Dynamic metaball contours with physics |
| 14 | Mixed Shapes | All shape types together |

### Controls

- **Right/Left Arrow**: Step simulation forward/backward
- **Enter**: Print world state
- **Bomb button**: Drop a random box from above
- **Sliders**: Adjust physics and metaball parameters in real-time

### Running the Collision Debugger

```bash
cd examples/collision-debug
cargo run --release
```

Arrow keys move the second body, Enter recalculates contacts.

### ASCII Collision Example

```bash
cargo run --example simple_collide
```

## Math Utilities

### Vec2

```rust
use sylt_2d::math_utils::Vec2;

let v = Vec2::new(1.0, 2.0);
let len = v.length();
let dot = v.dot(Vec2::new(3.0, 4.0));
let abs = v.abs();                    // component-wise absolute value

// Arithmetic: +, -, * (scalar), - (negation)
let sum = v + Vec2::new(1.0, 1.0);
let scaled = v * 2.0;
```

### Cross Product

```rust
use sylt_2d::math_utils::Cross;

let cross_f: f32 = Vec2::new(1.0, 0.0).cross(Vec2::new(0.0, 1.0)); // 1.0
let perp: Vec2 = Vec2::new(1.0, 0.0).cross(2.0);                    // (0, -2)
let perp2: Vec2 = 2.0f32.cross(Vec2::new(1.0, 0.0));                // (0, 2)
```

### Mat2x2

```rust
use sylt_2d::math_utils::Mat2x2;

let rot = Mat2x2::new_from_angle(std::f32::consts::PI / 4.0); // 45 degree rotation
let rotated = rot * Vec2::new(1.0, 0.0);
let inv = rot.invert().unwrap();
let m3 = rot * rot; // matrix multiplication
```

### AABB (Axis-Aligned Bounding Box)

```rust
use sylt_2d::math_utils::Aabb;

let aabb = Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0));
let expanded = aabb.expand(0.5);
let union = aabb.union(&other_aabb);
let center = aabb.center();
let overlaps = aabb.overlaps(&other);
```

## License

See source for details. Based on Box2D-lite by Erin Catto.
