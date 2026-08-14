//! Standalone repro: capture a ball on a flipper, then flip again, and observe
//! the ball ending up on the wrong side of the blade ("the ball goes through
//! the flipper").
//!
//! Run with:
//!   cargo test --test capture_flip_tunnel -- --nocapture
//!
//! What it builds: the pinball default-board geometry (cabinet walls, inlane
//! guides, plunger divider, both flipper pivots with hinge joints, drain floor),
//! the exact constants from `examples/pinball/src/board.rs` / `state.rs`.
//!
//! What it reports per (flipper, capture spot):
//!   - `min_sep` : closest signed distance ball-vs-blade during the flip
//!     (surface-to-surface; < -0.35 would mean the ball CENTER is embedded —
//!     a true material tunnel). Tool threshold used by the game is 3.0 u/s
//!     (`World::set_ccd_speed_threshold(3.0)`), so a captured ball is slow and
//!     is never swept by CCD; only the fast flipper is.
//!   - TUNNELED: ball ended up below the blade line while clearly separated
//!     from it (0.3+ gap) i.e. it slipped over the blade tip into the drain.

use sylt_2d::body::Body;
use sylt_2d::joint::Joint;
use sylt_2d::math_utils::{Mat2x2, Vec2};
use sylt_2d::world::World;

const DT: f32 = 1.0 / 60.0;
const LENGTH: f32 = 2.2;
const WIDTH: f32 = 0.55;
const HALF: Vec2 = Vec2 {
    x: LENGTH / 2.0,
    y: WIDTH / 2.0,
};
const BALL_R: f32 = 0.35;
const CCD_THRESHOLD: f32 = 3.0;

const LEFT_PIVOT: Vec2 = Vec2 { x: -3.2, y: -0.6 };
const RIGHT_PIVOT: Vec2 = Vec2 { x: 1.6, y: -0.6 };
const LEFT_REST: f32 = -0.4;
const RIGHT_REST: f32 = 0.4;
const LEFT_UP: f32 = LEFT_REST + 0.9;
const RIGHT_UP: f32 = RIGHT_REST - 0.9;

fn find(world: &World, id: usize) -> (Vec2, Vec2, f32) {
    for b in world.bodies.iter() {
        let b = b.borrow();
        if b.id == id {
            return (b.position, b.velocity, b.rotation);
        }
    }
    panic!("body {} not found", id);
}

/// Replicates `physics::drive_flipper`: a first-order servo that sets angular
/// velocity AND the matching orbital linear velocity about the pivot, with the
/// settle dead-zone. Same gain/clamp as the game (per-frame rotation capped).
fn drive_flipper(body: &mut Body, target: f32, side: i32) {
    const G: f32 = 60.0;
    const MAX_PER_FRAME: f32 = 0.25;
    const SETTLE_ANGLE: f32 = 0.03;
    const SETTLE_SPEED: f32 = 0.25;
    let offset = Vec2 {
        x: LENGTH * 0.45 * side as f32,
        y: 0.0,
    };
    let err = target - body.rotation;
    let max_av = MAX_PER_FRAME / DT;
    let av = (err * G).clamp(-max_av, max_av);
    body.angular_velocity = av;
    let c = offset.x * body.rotation.cos() - offset.y * body.rotation.sin();
    let s = offset.x * body.rotation.sin() + offset.y * body.rotation.cos();
    let pivot = body.position - Vec2 { x: c, y: s };
    let to_center = body.position - pivot;
    body.velocity = Vec2 {
        x: -to_center.y,
        y: to_center.x,
    } * av;
    if err.abs() < SETTLE_ANGLE && av.abs() < SETTLE_SPEED {
        body.angular_velocity = 0.0;
        body.velocity = Vec2::new(0.0, 0.0);
        body.rotation = target;
    }
}

/// Signed surface distance, ball vs rotated blade. Negative = touching/inside.
fn sep(circle_pos: Vec2, r: f32, box_pos: Vec2, box_rot: f32) -> f32 {
    let rot_t = Mat2x2::new_from_angle(-box_rot);
    let local = rot_t * (circle_pos - box_pos);
    let abs = local.abs();
    let dx = abs.x - HALF.x;
    let dy = abs.y - HALF.y;
    if dx > 0.0 || dy > 0.0 {
        Vec2::new(dx.max(0.0), dy.max(0.0)).length() - r
    } else {
        dx.max(dy) - r
    }
}

fn static_box(world: &mut World, w: Vec2, pos: Vec2, rot: f32, friction: f32) {
    let mut b = Body::new(w, f32::MAX);
    b.position = pos;
    b.rotation = rot;
    b.friction = friction;
    world.add_body(b);
}

fn spawn_flipper(world: &mut World, pivot: Vec2, rest: f32, side: i32) -> usize {
    let rot = Mat2x2::new_from_angle(rest);
    let mut flipper = Body::new(Vec2::new(LENGTH, WIDTH), 15.0);
    flipper.position = pivot
        + rot
            * Vec2 {
                x: LENGTH * 0.45 * side as f32,
                y: 0.0,
            };
    flipper.friction = 0.6;
    flipper.rotation = rest;
    let fid = flipper.id;
    world.add_body(flipper.clone());

    let mut anchor = Body::new(Vec2::new(0.1, 0.1), f32::MAX);
    anchor.position = pivot;
    world.add_body(anchor.clone());

    let mut joint = Joint::new(anchor, flipper, pivot, &world);
    joint.softness = 0.005;
    world.add_joint(joint);
    fid
}

fn build_default_board() -> (World, usize, usize) {
    let mut world = World::new(Vec2::new(0.0, -15.0), 120);
    world.set_ccd_enabled(true);
    world.set_ccd_speed_threshold(CCD_THRESHOLD);
    world.set_restitution(0.35);

    // ``board.rs`` cabinet walls + guides + divider.
    static_box(
        &mut world,
        Vec2::new(20.0, 1.0),
        Vec2::new(0.0, -5.0),
        0.0,
        0.2,
    ); // floor
    static_box(
        &mut world,
        Vec2::new(0.6, 26.0),
        Vec2::new(-8.3, 9.0),
        0.0,
        0.05,
    ); // left wall
    static_box(
        &mut world,
        Vec2::new(0.6, 26.0),
        Vec2::new(8.3, 9.0),
        0.0,
        0.05,
    ); // right wall
    static_box(
        &mut world,
        Vec2::new(0.5, 17.0),
        Vec2::new(6.1, 4.5),
        0.0,
        0.05,
    ); // divider
    static_box(
        &mut world,
        Vec2::new(5.7, 0.8),
        Vec2::new(-5.86, 1.99),
        -0.55,
        0.1,
    ); // left inlane
    static_box(
        &mut world,
        Vec2::new(5.0, 0.8),
        Vec2::new(3.6, 1.8),
        0.55,
        0.1,
    ); // right inlane

    let lf = spawn_flipper(&mut world, LEFT_PIVOT, LEFT_REST, 1);
    let rf = spawn_flipper(&mut world, RIGHT_PIVOT, RIGHT_REST, -1);
    (world, lf, rf)
}

struct Result {
    captured: f32,
    min_sep: f32,
    tunneled: bool,
    how: String,
    end: Vec2,
}

fn run_case(side: &str, spot: &str) -> Result {
    let (mut world, lf, rf) = build_default_board();

    let (pivot, rest, target, side_sign, fid) = match side {
        "left" => (LEFT_PIVOT, LEFT_REST, LEFT_UP, 1, lf),
        _ => (RIGHT_PIVOT, RIGHT_REST, RIGHT_UP, -1, rf),
    };
    let lx = match spot {
        "root" => -0.6,
        "mid" => 0.0,
        "tip" => 0.75,
        _ => 0.0,
    };
    let ly = 0.35 + WIDTH / 2.0 + 0.02;

    let mut ball = Body::new_circle(BALL_R, 1.5);
    ball.position = pivot + Mat2x2::new_from_angle(rest) * Vec2 { x: lx, y: ly };
    ball.velocity = Vec2::new(0.0, 0.0);
    let bid = ball.id;
    world.add_body(ball);

    // Phase 1 — capture: hold both blades at rest long enough to establish
    // solid ball-on-blade contact.
    for _ in 0..15 {
        for b in world.bodies.iter() {
            let id = b.borrow().id;
            if id == lf {
                drive_flipper(&mut b.borrow_mut(), LEFT_REST, 1);
            } else if id == rf {
                drive_flipper(&mut b.borrow_mut(), RIGHT_REST, -1);
            }
        }
        world.step(DT).unwrap();
    }
    let (bp0, _, _) = find(&world, bid);
    let (fp0, _, fr0) = find(&world, fid);
    let captured_sep = sep(bp0, BALL_R, fp0, fr0);

    // Phase 2 — flip again.
    let mut min_sep = f32::MAX;
    let mut tunneled = false;
    let mut how = String::new();
    for f in 0..100 {
        for b in world.bodies.iter() {
            let id = b.borrow().id;
            if id == fid {
                drive_flipper(&mut b.borrow_mut(), target, side_sign);
            } else if id == lf {
                drive_flipper(&mut b.borrow_mut(), LEFT_REST, 1);
            } else if id == rf {
                drive_flipper(&mut b.borrow_mut(), RIGHT_REST, -1);
            }
        }
        world.step(DT).unwrap();
        let (bp, _, _) = find(&world, bid);
        let (fp, _, fr) = find(&world, fid);
        let s = sep(bp, BALL_R, fp, fr);
        min_sep = min_sep.min(s);

        // True material tunnel: ball center embedded deeper than its radius.
        if s < -BALL_R && !tunneled {
            tunneled = true;
            how = format!("CENTER EMBEDDED sep={:.3} at f={}", s, f);
        }
        // The behavior reported by players: ball slides off the blade tip and
        // drops below it — separated from the blade but on the drain side.
        {
            let rot_t = Mat2x2::new_from_angle(-fr);
            let local = rot_t * (bp - fp);
            if local.x > HALF.x + 0.2 && local.y < -0.2 && s > 0.3 && !tunneled {
                tunneled = true;
                how = format!(
                    "rode over TIP, dropped below blade local=({:.2},{:.2}) at f={}",
                    local.x, local.y, f
                );
            }
        }
        if bp.y < -2.5 && s > 0.5 && !tunneled {
            tunneled = true;
            how = format!("fell to floor past blade, sep={:.2} at f={}", s, f);
        }
        if tunneled {
            break;
        }
    }
    let (bp_end, _, _) = find(&world, bid);
    Result {
        captured: captured_sep,
        min_sep,
        tunneled,
        how,
        end: bp_end,
    }
}

#[test]
fn capture_then_flip_ball_must_not_end_wrong_side() {
    println!("CCD threshold in the game = {CCD_THRESHOLD} u/s; captured ball is slow => never swept by CCD.");
    println!("flip is driven by the servo (`drive_flipper`), blade turns up to ~0.7 rad/frame.");
    println!("sep < -0.35  => ball center embedded in the blade (true tunnel).");
    println!("TUNNELED     => ball ended below the blade line, separated from it.");
    println!("left flipper: rest -0.4 -> up +0.5   right flipper: rest +0.4 -> up -0.5\n");

    let mut embed_any = false;
    for side in ["left", "right"] {
        for spot in ["root", "mid", "tip"] {
            let r = run_case(side, spot);
            if r.min_sep < -BALL_R {
                embed_any = true;
            }
            println!(
                "[{:<5} {:<4}] captured={:.3} min_sep={:.3} end=({:>5.2},{:>6.2})  {}",
                side,
                spot,
                r.captured,
                r.min_sep,
                r.end.x,
                r.end.y,
                if r.tunneled {
                    format!("TUNNELED: {}", r.how)
                } else {
                    "launched OK".to_string()
                }
            );
        }
    }
    println!();
    assert!(
        !embed_any,
        "ball center embedded in a blade (min_sep < -{BALL_R}): true material tunneling"
    );
}
