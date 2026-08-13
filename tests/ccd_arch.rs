use std::cell::Ref;

use sylt_2d::body::Body;
use sylt_2d::math_utils::Vec2;
use sylt_2d::world::World;

const DT: f32 = 1.0 / 60.0;
const BALL_RADIUS: f32 = 0.35;
const ARC_CENTER: Vec2 = Vec2 { x: -0.5, y: 17.0 };
const R_OUTER: f32 = 8.8;
const R_INNER: f32 = 8.2;

fn find_body<'a>(world: &'a World, id: usize) -> Ref<'a, Body> {
    let body = world
        .bodies
        .iter()
        .find(|b| b.borrow().id == id)
        .expect("body not found");
    body.borrow()
}

/// Builds the pinball's top main curved arch: 12 trapezoid convex-polygon
/// segments, world-space vertices, static, friction 0.05. Returns the world and
/// the ids of the arch segments.
fn arch_world() -> (World, Vec<usize>) {
    let mut world = World::new(Vec2::new(0.0, 0.0), 40);
    let mut ids = Vec::new();
    for i in 0..12usize {
        let theta1 = std::f32::consts::PI * (i as f32 / 12.0);
        let theta2 = std::f32::consts::PI * ((i + 1) as f32 / 12.0);
        let p1_inner = ARC_CENTER + Vec2::new(R_INNER * theta1.cos(), R_INNER * theta1.sin());
        let p2_inner = ARC_CENTER + Vec2::new(R_INNER * theta2.cos(), R_INNER * theta2.sin());
        let p2_outer = ARC_CENTER + Vec2::new(R_OUTER * theta2.cos(), R_OUTER * theta2.sin());
        let p1_outer = ARC_CENTER + Vec2::new(R_OUTER * theta1.cos(), R_OUTER * theta1.sin());
        let mut seg = Body::new_polygon(vec![p1_inner, p2_inner, p2_outer, p1_outer], f32::MAX);
        seg.friction = 0.05;
        let id = seg.id;
        world.add_body(seg);
        ids.push(id);
    }
    (world, ids)
}

/// Signed distance from a point to a convex polygon (negative inside),
/// computed in world space.
fn point_poly_dist(p: Vec2, body: &Body) -> f32 {
    let poly = body
        .get_polygon()
        .rotate(body.rotation)
        .translate(body.position);
    let n = poly.get_num_vertices();
    let mut inside = true;
    let mut min_dist = f32::MAX;
    for i in 0..n {
        let a = poly.get_vertex(i as isize);
        let b = poly.get_vertex(i as isize + 1);
        let e = b - a;
        let e_len = e.length();
        if e_len < 1e-6 {
            continue;
        }
        let nrm = Vec2::new(e.y, -e.x) * (1.0 / e_len);
        if (p - a).dot(nrm) < 0.0 {
            inside = false;
        }
        let t = ((p - a).dot(e) / e_len).clamp(0.0, 1.0);
        min_dist = min_dist.min((p - (a + e * t)).length());
    }
    if inside {
        -min_dist
    } else {
        min_dist
    }
}

/// Minimum signed separation between the ball and every arch segment.
/// Positive = separated, negative = penetrating the arch material.
fn min_arch_sep(world: &World, ball_id: usize, seg_ids: &[usize]) -> f32 {
    let ball = find_body(world, ball_id);
    seg_ids
        .iter()
        .map(|id| point_poly_dist(ball.position, &find_body(world, *id)) - BALL_RADIUS)
        .fold(f32::MAX, f32::min)
}

struct Outcome {
    min_sep: f32,
    min_sep_frame: usize,
    ball_final: Vec2,
    speed_final: f32,
}

/// Steps a ball against the arch for `frames` steps, tracking the minimum
/// separation from the arch material.
fn run_arch_ball(start: Vec2, vel: Vec2, frames: usize) -> (Outcome, World) {
    let (mut world, ids) = arch_world();
    let mut ball = Body::new_circle(BALL_RADIUS, 1.5);
    ball.position = start;
    ball.velocity = vel;
    let bid = ball.id;
    world.add_body(ball);

    let mut min_sep = f32::MAX;
    let mut min_sep_frame = 0;
    for f in 0..frames {
        world.step(DT).unwrap();
        let sep = min_arch_sep(&world, bid, &ids);
        if sep < min_sep {
            min_sep = sep;
            min_sep_frame = f;
        }
    }
    let o = {
        let ball = find_body(&world, bid);
        Outcome {
            min_sep,
            min_sep_frame,
            ball_final: ball.position,
            speed_final: ball.velocity.length(),
        }
    };
    (o, world)
}

// ---------------------------------------------------------------------------
// T1 (stuck-in-arc repro) Ball launched straight up into the arch crown. It
// must bounce off the inner face — not embed inside the arch material and not
// tunnel through the crown.
// ---------------------------------------------------------------------------
#[test]
fn crown_headon_no_stick() {
    let (o, _world) = run_arch_ball(Vec2::new(0.0, 6.0), Vec2::new(0.0, 60.0), 100);
    println!(
        "  ball_final=({:.3},{:.3}) speed={:.3} min_sep={:.4} at frame {}",
        o.ball_final.x, o.ball_final.y, o.speed_final, o.min_sep, o.min_sep_frame
    );
    // Ball must not be embedded deep in the arch material (radius is 0.35;
    // anything beyond a fraction of a radius means it stuck inside the arc).
    assert!(
        o.min_sep > -0.3,
        "ball stuck inside the arch material, min_sep={}",
        o.min_sep
    );
    // The ball must not rest stuck on/inside the crown: it should bounce off
    // and leave the crown area (in particular never sit above the crown face).
    assert!(
        o.ball_final.y < ARC_CENTER.y + R_OUTER,
        "ball ended above the arch crown, y={}",
        o.ball_final.y
    );
}

// ---------------------------------------------------------------------------
// T2 Fast ball (1.5 units/frame) straight into the crown. Same guarantees at
// higher speed, where a face test with the wrong offset is most likely to miss.
// ---------------------------------------------------------------------------
#[test]
fn crown_headon_fast() {
    let (o, _world) = run_arch_ball(Vec2::new(-0.5, 6.0), Vec2::new(0.0, 90.0), 120);
    println!(
        "  ball_final=({:.3},{:.3}) speed={:.3} min_sep={:.4} at frame {}",
        o.ball_final.x, o.ball_final.y, o.speed_final, o.min_sep, o.min_sep_frame
    );
    assert!(
        o.min_sep > -0.3,
        "ball stuck inside the arch material at high speed, min_sep={}",
        o.min_sep
    );
    assert!(
        o.ball_final.y < ARC_CENTER.y + R_OUTER,
        "ball ended above the arch crown, y={}",
        o.ball_final.y
    );
}

// ---------------------------------------------------------------------------
// T3 Ball shot from the lower right into the steep underside of the arch.
// ---------------------------------------------------------------------------
#[test]
fn underside_steep_right() {
    let (o, _world) = run_arch_ball(Vec2::new(5.0, 3.0), Vec2::new(15.0, 70.0), 120);
    println!(
        "  ball_final=({:.3},{:.3}) speed={:.3} min_sep={:.4} at frame {}",
        o.ball_final.x, o.ball_final.y, o.speed_final, o.min_sep, o.min_sep_frame
    );
    assert!(
        o.min_sep > -0.3,
        "ball penetrated the arch underside, min_sep={}",
        o.min_sep
    );
}
