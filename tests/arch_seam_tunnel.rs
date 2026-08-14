//! Standalone repro: high-speed ball hitting the segmented top arch at the seam
//! between two arch segments can tunnel through the shell.
//!
//! Rebuilds the game container: the cabinet side walls (x=+-8.3, width 0.6) and
//! the top arch (12 overlapping convex quadrilateral segments, inner radius 8.2,
//! outer radius 9.4, center (-0.5, 17.0)) exactly as `board.rs`. CCD is enabled
//! like the game (threshold 3.0, restitution 0.35). A ball is slammed at the
//! inner face at high speed; a tunnel is only counted when the ball actually
//! pierces the arch shell (beyond the outer radius while still within the arch's
//! angular span) or exits through a side wall.

use sylt_2d::body::Body;
use sylt_2d::math_utils::Vec2;
use sylt_2d::world::World;

const DT: f32 = 1.0 / 60.0;
const BALL_R: f32 = 0.35;
const BALL_MASS: f32 = 1.0;
const GRAVITY: Vec2 = Vec2 { x: 0.0, y: -15.0 };
const ITERATIONS: u32 = 10;

const ARCH_CENTER: Vec2 = Vec2 { x: -0.5, y: 17.0 };
const NUM_SEGMENTS: usize = 12;
const RADIUS_OUTER: f32 = 9.4;
const RADIUS_INNER: f32 = 8.2;
const OVERLAP_FACTOR: f32 = 1.6;

const WALL_X: f32 = 8.3;
const WALL_HALF: f32 = 0.3;

fn build_arch(world: &mut World) {
    let seg_span = std::f32::consts::PI / NUM_SEGMENTS as f32;
    for i in 0..NUM_SEGMENTS {
        let theta_mid = std::f32::consts::PI * (i as f32 + 0.5) / NUM_SEGMENTS as f32;
        let theta1 = theta_mid - seg_span * OVERLAP_FACTOR / 2.0;
        let theta2 = theta_mid + seg_span * OVERLAP_FACTOR / 2.0;
        let p1_inner =
            ARCH_CENTER + Vec2::new(RADIUS_INNER * theta1.cos(), RADIUS_INNER * theta1.sin());
        let p2_inner =
            ARCH_CENTER + Vec2::new(RADIUS_INNER * theta2.cos(), RADIUS_INNER * theta2.sin());
        let p2_outer =
            ARCH_CENTER + Vec2::new(RADIUS_OUTER * theta2.cos(), RADIUS_OUTER * theta2.sin());
        let p1_outer =
            ARCH_CENTER + Vec2::new(RADIUS_OUTER * theta1.cos(), RADIUS_OUTER * theta1.sin());
        let poly_verts = vec![p1_inner, p2_inner, p2_outer, p1_outer];
        let mut arch_seg = Body::new_polygon(poly_verts, f32::MAX);
        arch_seg.friction = 0.05;
        world.add_body(arch_seg);
    }
}

fn add_side_walls(world: &mut World) {
    let mut lw = Body::new(Vec2::new(WALL_HALF, 26.0), f32::MAX);
    lw.position = Vec2::new(-WALL_X, 9.0);
    lw.friction = 0.05;
    world.add_body(lw);
    let mut rw = Body::new(Vec2::new(WALL_HALF, 26.0), f32::MAX);
    rw.position = Vec2::new(WALL_X, 9.0);
    rw.friction = 0.05;
    world.add_body(rw);
    let mut floor = Body::new(Vec2::new(14.0, 0.5), f32::MAX);
    floor.position = Vec2::new(0.0, -2.0);
    floor.friction = 0.05;
    world.add_body(floor);
}

fn add_ball(world: &mut World, pos: Vec2, vel: Vec2) -> usize {
    let mut ball = Body::new_circle(BALL_R, BALL_MASS);
    ball.position = pos;
    ball.velocity = vel;
    let id = ball.id;
    world.add_body(ball);
    id
}

fn ball_state(world: &World, id: usize) -> (Vec2, Vec2) {
    for b in world.bodies.iter() {
        let b = b.borrow();
        if b.id == id {
            return (b.position, b.velocity);
        }
    }
    panic!("ball body not found");
}

fn in_arch_span(theta: f32) -> bool {
    theta > 0.02 && theta < std::f32::consts::PI - 0.02
}

#[test]
fn high_speed_ball_must_not_tunnel_arch() {
    println!(
        "arch: {} segments, overlap x{:.1}, r_in={} r_out={}, center=({}, {})",
        NUM_SEGMENTS, OVERLAP_FACTOR, RADIUS_INNER, RADIUS_OUTER, ARCH_CENTER.x, ARCH_CENTER.y
    );
    println!(
        "ball radius {} at speeds aimed at the inner face (incl. seams):",
        BALL_R
    );

    let mut any_tunnel = false;
    for speed in [20.0f32, 30.0, 40.0, 55.0] {
        for i in 0..NUM_SEGMENTS * 2 {
            // aim at every segment midpoint and every seam around the arch
            let frac = (i as f32 + 0.5) / (NUM_SEGMENTS * 2) as f32;
            let theta = std::f32::consts::PI * frac;
            let dir = Vec2::new(theta.cos(), theta.sin());
            let start = ARCH_CENTER + dir * (RADIUS_INNER - 1.2);
            let vel = dir * speed;

            let mut world = World::new(GRAVITY, ITERATIONS);
            world.set_ccd_enabled(true);
            world.set_ccd_speed_threshold(3.0);
            world.set_restitution(0.35);
            build_arch(&mut world);
            add_side_walls(&mut world);
            let ball = add_ball(&mut world, start, vel);

            let mut tunneled = false;
            let mut how = "";
            let mut frames = 0;
            for _ in 0..300 {
                let _ = world.step(DT);
                frames += 1;
                let (p, _) = ball_state(&world, ball);
                let rel = p - ARCH_CENTER;
                let r = rel.length();
                let theta_p = rel.y.atan2(rel.x);
                if in_arch_span(theta_p) && r > RADIUS_OUTER + 0.15 {
                    tunneled = true;
                    how = "THROUGH SHELL";
                    break;
                }
                if p.y < -2.5 {
                    // Below the cabinet floor: in-game the ball hits the drain
                    // and is respawned (not a tunnel). Stop tracking.
                    break;
                }
                if p.x > WALL_X + WALL_HALF + 0.3 || p.x < -(WALL_X + WALL_HALF + 0.3) {
                    tunneled = true;
                    how = "THROUGH WALL";
                    break;
                }
            }

            let (p, _) = ball_state(&world, ball);
            if tunneled {
                any_tunnel = true;
                println!("  v={speed:>3} hit@theta={:.3} -> TUNNELED ({how}) at f={frames} pos=({:+.2},{:+.2})",
                    theta, p.x, p.y);
            }
        }
        println!("  (v={speed} done)");
    }

    assert!(
        !any_tunnel,
        "ball tunneled through the arch shell or a side wall at high speed"
    );
}
