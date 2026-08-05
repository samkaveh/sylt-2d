use std::cell::Ref;

use sylt_2d::body::{Body, Shape};
use sylt_2d::joint::Joint;
use sylt_2d::math_utils::{Mat2x2, Vec2};
use sylt_2d::world::World;

const DT: f32 = 1.0 / 60.0;

fn find_body<'a>(world: &'a World, id: usize) -> Ref<'a, Body> {
    let body = world
        .bodies
        .iter()
        .find(|b| b.borrow().id == id)
        .expect("body not found");
    body.borrow()
}

fn pos(world: &World, id: usize) -> Vec2 {
    find_body(world, id).position
}

/// Signed distance between a circle and a (possibly rotated) box.
/// Positive = separated, negative = penetrating.
fn sep_circle_box(circle_pos: Vec2, circle_r: f32, box_pos: Vec2, box_rot: f32, half: Vec2) -> f32 {
    let rot_t = Mat2x2::new_from_angle(-box_rot);
    let local = rot_t * (circle_pos - box_pos);
    let abs = local.abs();
    let dx = abs.x - half.x;
    let dy = abs.y - half.y;
    if dx > 0.0 || dy > 0.0 {
        Vec2::new(dx.max(0.0), dy.max(0.0)).length() - circle_r
    } else {
        dx.max(dy) - circle_r
    }
}

fn sep_circle_circle(a: Vec2, ra: f32, b: Vec2, rb: f32) -> f32 {
    (a - b).length() - (ra + rb)
}

fn obstacle_sep(world: &World, ball_id: usize, obstacle_id: usize) -> f32 {
    let ball = find_body(world, ball_id);
    let ob = find_body(world, obstacle_id);
    match ob.shape {
        Shape::Circle => sep_circle_circle(ball.position, ball.radius, ob.position, ob.radius),
        _ => sep_circle_box(
            ball.position,
            ball.radius,
            ob.position,
            ob.rotation,
            ob.width * 0.5,
        ),
    }
}

struct Outcome {
    min_sep: f32,
    min_sep_frame: usize,
    ball_final: Vec2,
    obstacle_final: Vec2,
    samples: Vec<(usize, Vec2, Vec2, f32)>,
}

/// Step `frames` times, tracking ball-obstacle separation each frame.
fn run_outcome(world: &mut World, ball_id: usize, obstacle_id: usize, frames: usize) -> Outcome {
    let mut min_sep = f32::MAX;
    let mut min_sep_frame = 0;
    let mut samples = Vec::with_capacity(frames);
    for f in 0..frames {
        world.step(DT).unwrap();
        let b = pos(world, ball_id);
        let o = pos(world, obstacle_id);
        let sep = obstacle_sep(world, ball_id, obstacle_id);
        if sep < min_sep {
            min_sep = sep;
            min_sep_frame = f;
        }
        samples.push((f, b, o, sep));
    }
    Outcome {
        min_sep,
        min_sep_frame,
        ball_final: samples.last().map(|s| s.1).unwrap(),
        obstacle_final: samples.last().map(|s| s.2).unwrap(),
        samples,
    }
}

fn print_trace(o: &Outcome) {
    println!(
        "  ball_final=({:.3},{:.3}) obstacle_final=({:.3},{:.3})",
        o.ball_final.x, o.ball_final.y, o.obstacle_final.x, o.obstacle_final.y
    );
    println!("  min_sep={:.4} at frame {}", o.min_sep, o.min_sep_frame);
    for (f, b, ob, sep) in &o.samples {
        if f % 3 == 0 || *sep < -0.1 {
            println!(
                "  f{:>3} ball=({:7.3},{:7.3}) ob=({:7.3},{:7.3}) sep={:8.4}",
                f, b.x, b.y, ob.x, ob.y, sep
            );
        }
    }
}

// ---------------------------------------------------------------------------
// T1 (control) Static flipper, ball into the flat face. Should not tunnel.
// ---------------------------------------------------------------------------
#[test]
fn control_flipper_face() {
    let mut world = World::new(Vec2::new(0.0, 0.0), 40);
    let mut flipper = Body::new(Vec2::new(2.2, 0.45), f32::MAX);
    flipper.position = Vec2::new(0.0, -1.0);
    let fid = flipper.id;
    world.add_body(flipper);

    let mut ball = Body::new_circle(0.35, 1.5);
    ball.position = Vec2::new(0.0, 1.5);
    ball.velocity = Vec2::new(0.0, -60.0);
    let bid = ball.id;
    world.add_body(ball);

    let o = run_outcome(&mut world, bid, fid, 12);
    print_trace(&o);
    // Flipper top at y = -0.775; a reflected ball stays above -1.0.
    assert!(
        o.ball_final.y > -1.0,
        "ball tunneled through flipper face, final y={}",
        o.ball_final.y
    );
    assert!(
        o.min_sep > -0.3,
        "ball deeply penetrated flipper face, min_sep={}",
        o.min_sep
    );
}

// ---------------------------------------------------------------------------
// T2 Static flipper rotated 45°, ball dropped straight onto the tip vertex.
// ---------------------------------------------------------------------------
#[test]
fn flipper_tip_corner() {
    let mut world = World::new(Vec2::new(0.0, 0.0), 40);
    let mut flipper = Body::new(Vec2::new(2.2, 0.45), f32::MAX);
    flipper.position = Vec2::new(0.0, 0.0);
    flipper.rotation = std::f32::consts::FRAC_PI_4;
    let fid = flipper.id;
    world.add_body(flipper);

    // Top-right vertex of the rotated box is at (0.619, 0.937).
    let mut ball = Body::new_circle(0.35, 1.5);
    ball.position = Vec2::new(0.619, 1.5);
    ball.velocity = Vec2::new(0.0, -60.0);
    let bid = ball.id;
    world.add_body(ball);

    let o = run_outcome(&mut world, bid, fid, 12);
    print_trace(&o);
    // A ball that bounces off the tip stays well above the flipper.
    assert!(
        o.ball_final.y > 0.5,
        "ball tunneled through flipper tip, final y={}",
        o.ball_final.y
    );
    assert!(
        o.min_sep > -0.3,
        "ball deeply penetrated flipper tip, min_sep={}",
        o.min_sep
    );
}

// ---------------------------------------------------------------------------
// T3 Dynamic flipper anchored at a pivot, spinning at 28 rad/s, ball drops in.
// ---------------------------------------------------------------------------
#[test]
fn rotating_flipper_swing() {
    let mut world = World::new(Vec2::new(0.0, 0.0), 40);
    let pivot = Vec2::new(-3.2, -0.6);
    let init_rot = -0.4f32;

    let mut flipper = Body::new(Vec2::new(2.2, 0.45), 100.0);
    let rot_mat = Mat2x2::new_from_angle(init_rot);
    flipper.position = pivot + rot_mat * Vec2::new(2.2 * 0.45, 0.0);
    flipper.rotation = init_rot;
    flipper.angular_velocity = 28.0;
    let fid = flipper.id;
    world.add_body(flipper.clone());

    let mut anchor = Body::new(Vec2::new(0.1, 0.1), f32::MAX);
    anchor.position = pivot;
    world.add_body(anchor.clone());

    let mut joint = Joint::new(anchor, flipper, pivot, &world);
    joint.softness = 0.02;
    world.add_joint(joint);

    let mut ball = Body::new_circle(0.35, 1.5);
    ball.position = Vec2::new(-2.6, 1.5);
    ball.velocity = Vec2::new(0.0, -60.0);
    let bid = ball.id;
    world.add_body(ball);

    let o = run_outcome(&mut world, bid, fid, 20);
    print_trace(&o);
    // Ball starts above the flipper; if the swing is handled it must stay above.
    assert!(
        o.ball_final.y > -1.0,
        "ball tunneled through swinging flipper, final y={}",
        o.ball_final.y
    );
    assert!(
        o.min_sep > -0.4,
        "ball deeply penetrated swinging flipper, min_sep={}",
        o.min_sep
    );
}

// ---------------------------------------------------------------------------
// T4 (control) Thin dynamic plank at rest, ball straight onto the face.
// ---------------------------------------------------------------------------
#[test]
fn control_plank_face_headon() {
    let mut world = World::new(Vec2::new(0.0, 0.0), 40);
    let mut plank = Body::new(Vec2::new(0.38, 0.2), 1.0);
    plank.position = Vec2::new(0.0, 0.0);
    let pid = plank.id;
    world.add_body(plank);

    let mut ball = Body::new_circle(0.35, 1.5);
    ball.position = Vec2::new(0.0, 1.0);
    ball.velocity = Vec2::new(0.0, -60.0);
    let bid = ball.id;
    world.add_body(ball);

    let o = run_outcome(&mut world, bid, pid, 15);
    print_trace(&o);
    assert!(
        o.ball_final.y > o.obstacle_final.y - 0.1,
        "ball passed through resting plank, ball_y={} plank_y={}",
        o.ball_final.y,
        o.obstacle_final.y
    );
    assert!(
        o.min_sep > -0.3,
        "ball deeply penetrated resting plank, min_sep={}",
        o.min_sep
    );
}

// ---------------------------------------------------------------------------
// T5 Fast plank (v=15 >= threshold) meeting a fast ball (v=-60).
// ---------------------------------------------------------------------------
#[test]
fn fast_moving_plank_pair() {
    let mut world = World::new(Vec2::new(0.0, 0.0), 40);
    let mut plank = Body::new(Vec2::new(0.38, 0.2), 1.0);
    plank.position = Vec2::new(0.0, 0.0);
    plank.velocity = Vec2::new(0.0, 15.0);
    let pid = plank.id;
    world.add_body(plank);

    let mut ball = Body::new_circle(0.35, 1.5);
    ball.position = Vec2::new(0.0, 0.6);
    ball.velocity = Vec2::new(0.0, -60.0);
    let bid = ball.id;
    world.add_body(ball);

    let o = run_outcome(&mut world, bid, pid, 12);
    print_trace(&o);
    assert!(
        o.ball_final.y > o.obstacle_final.y - 0.1,
        "ball passed through fast-moving plank, ball_y={} plank_y={}",
        o.ball_final.y,
        o.obstacle_final.y
    );
    assert!(
        o.min_sep > -0.3,
        "ball deeply penetrated fast-moving plank, min_sep={}",
        o.min_sep
    );
}

// ---------------------------------------------------------------------------
// T6 Static tilted plank (0.3 rad), ball dropped onto the end corner.
// ---------------------------------------------------------------------------
#[test]
fn tilted_plank_corner() {
    let mut world = World::new(Vec2::new(0.0, 0.0), 40);
    let mut plank = Body::new(Vec2::new(0.38, 0.2), f32::MAX);
    plank.position = Vec2::new(0.0, 0.0);
    plank.rotation = 0.3;
    let pid = plank.id;
    world.add_body(plank);

    // Top-right vertex of the tilted plank is at approx (0.152, 0.151).
    let mut ball = Body::new_circle(0.35, 1.5);
    ball.position = Vec2::new(0.152, 1.0);
    ball.velocity = Vec2::new(0.0, -60.0);
    let bid = ball.id;
    world.add_body(ball);

    let o = run_outcome(&mut world, bid, pid, 12);
    print_trace(&o);
    assert!(
        o.ball_final.y > 0.2,
        "ball tunneled through tilted plank end, final y={}",
        o.ball_final.y
    );
    assert!(
        o.min_sep > -0.3,
        "ball deeply penetrated tilted plank, min_sep={}",
        o.min_sep
    );
}

// ---------------------------------------------------------------------------
// T7 (stress control) Very fast ball (90 u/s = 1.5 units/frame) into a thin
// static wall. High speed alone must not cause a tunnel.
// ---------------------------------------------------------------------------
#[test]
fn fast_ball_thin_static_wall() {
    let mut world = World::new(Vec2::new(0.0, 0.0), 40);
    let mut wall = Body::new(Vec2::new(10.0, 0.2), f32::MAX);
    wall.position = Vec2::new(0.0, 0.0);
    let wid = wall.id;
    world.add_body(wall);

    let mut ball = Body::new_circle(0.35, 1.5);
    ball.position = Vec2::new(0.0, 1.0);
    ball.velocity = Vec2::new(0.0, -90.0);
    let bid = ball.id;
    world.add_body(ball);

    let o = run_outcome(&mut world, bid, wid, 12);
    print_trace(&o);
    // Wall top at y = 0.1; a reflected ball stays above -0.3.
    assert!(
        o.ball_final.y > -0.3,
        "ball tunneled through thin static wall at high speed, final y={}",
        o.ball_final.y
    );
    assert!(
        o.min_sep > -0.3,
        "ball deeply penetrated thin static wall, min_sep={}",
        o.min_sep
    );
}

// ---------------------------------------------------------------------------
// T8 (control) Static bumper (circle) hit head-on by a fast ball.
// ---------------------------------------------------------------------------
#[test]
fn control_bumper_circle() {
    let mut world = World::new(Vec2::new(0.0, 0.0), 40);
    let mut bumper = Body::new_circle(0.8, f32::MAX);
    bumper.position = Vec2::new(0.0, 0.0);
    let bpid = bumper.id;
    world.add_body(bumper);

    let mut ball = Body::new_circle(0.35, 1.5);
    ball.position = Vec2::new(-2.5, 0.0);
    ball.velocity = Vec2::new(50.0, 0.0);
    let bid = ball.id;
    world.add_body(ball);

    let o = run_outcome(&mut world, bid, bpid, 12);
    print_trace(&o);
    assert!(
        o.ball_final.x < 0.5,
        "ball tunneled through bumper, final x={}",
        o.ball_final.x
    );
    assert!(
        o.min_sep > -0.3,
        "ball deeply penetrated bumper, min_sep={}",
        o.min_sep
    );
}
