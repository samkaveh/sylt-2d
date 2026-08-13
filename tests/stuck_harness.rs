use sylt_2d::body::Body;
use sylt_2d::joint::Joint;
use sylt_2d::math_utils::Vec2;
use sylt_2d::world::World;

const BALL_RADIUS: f32 = 0.35;
const FLIPPER_LENGTH: f32 = 2.2;
const FLIPPER_WIDTH: f32 = 0.45;
const DT: f32 = 1.0 / 60.0;

fn rotate_vec(v: Vec2, angle: f32) -> Vec2 {
    Vec2::new(
        v.x * angle.cos() - v.y * angle.sin(),
        v.x * angle.sin() + v.y * angle.cos(),
    )
}

/// Full left-flipper pocket geometry: outer wall, drain floor, flipper base
/// wall, inlane guide, plus the flipper+anchor+joint exactly as the example.
fn build_left_pocket(world: &mut World) -> usize {
    // Bottom Drain Floor
    let mut floor = Body::new(Vec2::new(20.0, 1.0), f32::MAX);
    floor.position = Vec2::new(0.0, -5.0);
    floor.friction = 0.2;
    world.add_body(floor);

    // Left Main Outer Wall
    let mut left_wall = Body::new(Vec2::new(0.6, 26.0), f32::MAX);
    left_wall.position = Vec2::new(-8.3, 9.0);
    left_wall.friction = 0.05;
    world.add_body(left_wall);

    // Slanted Left Inlane Guide
    let mut left_inlane = Body::new(Vec2::new(6.5, 0.5), f32::MAX);
    left_inlane.position = Vec2::new(-6.2, 2.2);
    left_inlane.rotation = -0.55;
    left_inlane.friction = 0.1;
    world.add_body(left_inlane);

    // Flipper at rest angle -0.4
    let pivot = Vec2::new(-3.2, -0.6);
    let offset_x = FLIPPER_LENGTH * 0.45;
    let rot_init = -0.4;
    let rot_mat = sylt_2d::math_utils::Mat2x2::new_from_angle(rot_init);
    let mut flipper = Body::new(Vec2::new(FLIPPER_LENGTH, FLIPPER_WIDTH), 15.0);
    flipper.position = pivot + rot_mat * Vec2::new(offset_x, 0.0);
    flipper.friction = 0.6;
    flipper.rotation = rot_init;
    let flipper_id = flipper.id;
    world.add_body(flipper.clone());

    let mut anchor = Body::new(Vec2::new(0.1, 0.1), f32::MAX);
    anchor.position = pivot;
    world.add_body(anchor.clone());

    let mut joint = Joint::new(anchor, flipper, pivot, world);
    joint.softness = 0.005;
    world.add_joint(joint);
    flipper_id
}

fn drive_flipper(body: &mut Body, target: f32) {
    const PROPORTIONAL_GAIN: f32 = 45.0;
    const MAX_SPEED: f32 = 34.0;
    const SETTLE_ANGLE: f32 = 0.015;
    const SETTLE_SPEED: f32 = 0.25;
    let pivot_offset = Vec2::new(FLIPPER_LENGTH * 0.45, 0.0);

    let err = target - body.rotation;
    let av = (err * PROPORTIONAL_GAIN).clamp(-MAX_SPEED, MAX_SPEED);
    body.angular_velocity = av;
    let pivot = body.position - rotate_vec(pivot_offset, body.rotation);
    let to_center = body.position - pivot;
    body.velocity = Vec2::new(-to_center.y, to_center.x) * av;
    if err.abs() < SETTLE_ANGLE && av.abs() < SETTLE_SPEED {
        body.angular_velocity = 0.0;
        body.velocity = Vec2::new(0.0, 0.0);
        body.rotation = target;
    }
}

fn speed(b: &Body) -> f32 {
    (b.velocity.x * b.velocity.x + b.velocity.y * b.velocity.y).sqrt()
}

/// Simulate one ball launch through the left pocket. Returns a verdict string.
fn run_launch(pos: Vec2, vel: Vec2, flip_up: bool, frames: u32) -> String {
    let mut world = World::new(Vec2::new(0.0, -18.0), 120);
    world.set_restitution(0.35);
    let flipper_id = build_left_pocket(&mut world);

    let mut ball = Body::new_circle(BALL_RADIUS, 1.5);
    ball.friction = 0.02;
    ball.position = pos;
    ball.velocity = vel;
    let ball_id = ball.id;
    world.add_body(ball);

    let mut last_speed = 0.0f32;
    let mut low_speed_frames = 0u32;
    let mut peak = 0.0f32;
    let mut stuck_frames = 0u32;
    let mut final_pos = Vec2::new(0.0, 0.0);

    for _ in 0..frames {
        if let Some(b) = world.bodies.iter().find(|b| b.borrow().id == flipper_id) {
            let mut b = b.borrow_mut();
            let target = if flip_up { -0.4 + 0.9 } else { -0.4 };
            drive_flipper(&mut b, target);
        }
        let _ = world.step(DT);
        let b = world
            .bodies
            .iter()
            .find(|b| b.borrow().id == ball_id)
            .unwrap();
        let b = b.borrow();
        let sp = speed(&b);
        peak = peak.max(sp);
        final_pos = b.position;
        // "Stuck": ball alive but barely moving for a long stretch.
        if sp < 0.35 {
            low_speed_frames += 1;
        } else {
            low_speed_frames = 0;
        }
        if low_speed_frames > 90 {
            stuck_frames += 1;
        }
        last_speed = sp;
        let _ = b;
    }

    format!(
        "peak={peak:.2} final_speed={last_speed:.2} stuck_frames={stuck_frames} final_pos=({:.2},{:.2})",
        final_pos.x, final_pos.y
    )
}

#[test]
fn fuzz_left_pocket() {
    let mut stuck: Vec<String> = Vec::new();
    let mut total = 0;
    for &flip in &[false, true] {
        // Ball coming down the inlane toward the flipper.
        for x in [-5.5, -4.5, -3.5] {
            for y in [2.0, 1.0, 0.0, -0.5] {
                for vx in [4.0, 7.0, 10.0, 13.0] {
                    let pos = Vec2::new(x, y);
                    let vel = Vec2::new(vx, -2.0);
                    let r = run_launch(pos, vel, flip, 480);
                    total += 1;
                    if r.contains("stuck_frames>") || r.contains("stuck_frames=0") && false {
                        stuck.push(r);
                    } else {
                        let n: u32 = r
                            .split("stuck_frames=")
                            .nth(1)
                            .and_then(|s| s.split(' ').next())
                            .and_then(|s| s.parse().ok())
                            .unwrap_or(0);
                        if n > 20 {
                            stuck.push(format!("pos=({x},{y}) vx={vx} flip={flip} {r}"));
                        }
                    }
                }
            }
        }
    }
    println!("total launches: {total}");
    if stuck.is_empty() {
        println!("no stuck cases found");
    } else {
        println!("STUCK CASES ({}):", stuck.len());
        for s in stuck.iter().take(40) {
            println!("  {s}");
        }
    }
}

/// Ball sitting on the flipper at rest should roll off the tip, not rest forever.
#[test]
fn ball_on_flipper_rolls_off() {
    let mut world = World::new(Vec2::new(0.0, -18.0), 120);
    world.set_restitution(0.35);
    let flipper_id = build_left_pocket(&mut world);

    let pivot = Vec2::new(-3.2, -0.6);
    let mut ball = Body::new_circle(BALL_RADIUS, 1.5);
    ball.friction = 0.02;
    // Place it on the flipper face (slightly above the surface).
    ball.position = pivot + rotate_vec(Vec2::new(0.4, 0.5), -0.4);
    let ball_id = ball.id;
    world.add_body(ball);

    let mut escaped_side = false;
    let mut escaped_down = false;
    for _ in 0..(8.0 / DT) as u32 {
        if let Some(b) = world.bodies.iter().find(|b| b.borrow().id == flipper_id) {
            let mut b = b.borrow_mut();
            drive_flipper(&mut b, -0.4);
        }
        let _ = world.step(DT);
        let b = world
            .bodies
            .iter()
            .find(|b| b.borrow().id == ball_id)
            .unwrap();
        let b = b.borrow();
        if b.position.y < -4.0 {
            escaped_down = true;
            break;
        }
        if b.position.x < -7.0 {
            escaped_side = true;
            break;
        }
        let _ = b;
    }
    let b = world
        .bodies
        .iter()
        .find(|b| b.borrow().id == ball_id)
        .unwrap();
    let b = b.borrow();
    println!(
        "escaped_down={escaped_down} escaped_side={escaped_side} final=({:.2},{:.2}) speed={:.2}",
        b.position.x,
        b.position.y,
        speed(&b)
    );
    assert!(
        escaped_down || escaped_side,
        "ball rested on the flipper and never rolled off"
    );
}
