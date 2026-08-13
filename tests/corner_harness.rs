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

fn build_dual_flipper(world: &mut World) -> (usize, usize) {
    let floor = {
        let mut f = Body::new(Vec2::new(20.0, 1.0), f32::MAX);
        f.position = Vec2::new(0.0, -5.0);
        f.friction = 0.2;
        f
    };
    world.add_body(floor);
    let lw = {
        let mut w = Body::new(Vec2::new(0.6, 26.0), f32::MAX);
        w.position = Vec2::new(-8.3, 9.0);
        w.friction = 0.05;
        w
    };
    world.add_body(lw);
    let rw = {
        let mut w = Body::new(Vec2::new(0.6, 26.0), f32::MAX);
        w.position = Vec2::new(8.3, 9.0);
        w.friction = 0.05;
        w
    };
    world.add_body(rw);

    let left = add_flipper(world, Vec2::new(-3.2, -0.6), -0.4, 1.0);
    let right = add_flipper(world, Vec2::new(1.6, -0.6), 0.4, -1.0);
    (left, right)
}

fn add_flipper(world: &mut World, pivot: Vec2, rest: f32, side: f32) -> usize {
    let offset_x = FLIPPER_LENGTH * 0.45 * side;
    let rot_mat = sylt_2d::math_utils::Mat2x2::new_from_angle(rest);
    let mut flipper = Body::new(Vec2::new(FLIPPER_LENGTH, FLIPPER_WIDTH), 15.0);
    flipper.position = pivot + rot_mat * Vec2::new(offset_x, 0.0);
    flipper.friction = 0.6;
    flipper.rotation = rest;
    let id = flipper.id;
    world.add_body(flipper.clone());
    let mut anchor = Body::new(Vec2::new(0.001, 0.001), f32::MAX);
    anchor.position = pivot;
    world.add_body(anchor.clone());
    let mut joint = Joint::new(anchor, flipper, pivot, world);
    joint.softness = 0.005;
    world.add_joint(joint);
    id
}

fn drive_flipper(body: &mut Body, target: f32, pivot_offset: Vec2) {
    const PROPORTIONAL_GAIN: f32 = 45.0;
    const MAX_SPEED: f32 = 34.0;
    const SETTLE_ANGLE: f32 = 0.015;
    const SETTLE_SPEED: f32 = 0.25;
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

fn rand(u: &mut u64) -> f32 {
    *u = u
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    ((*u >> 11) as f32) / ((1u64 << 53) as f32)
}

/// Mass-version: thousands of balls dropped at random positions across the
/// playfield, flippers held or idle, check each one eventually drains. Any ball
/// that stays up (y > -3) forever = stuck in walls/flippers.
#[test]
fn mass_drop_never_drains() {
    let mut rng = 4242u64;
    let mut stuck = 0usize;
    let mut total = 0usize;
    let mut samples: Vec<(f32, f32, f32)> = Vec::new();

    for _ in 0..3000 {
        let mut world = World::new(Vec2::new(0.0, -18.0), 120);
        world.set_restitution(0.35);
        let (left_id, right_id) = build_dual_flipper(&mut world);

        let x = -7.5 + rand(&mut rng) * 15.0;
        let y = -1.0 + rand(&mut rng) * 6.0;
        let vx = -2.0 + rand(&mut rng) * 4.0;
        let mut ball = Body::new_circle(BALL_RADIUS, 1.5);
        ball.friction = 0.02;
        ball.position = Vec2::new(x, y);
        ball.velocity = Vec2::new(vx, 0.0);
        let bid = ball.id;
        world.add_body(ball);

        let flip_mode = rand(&mut rng);
        let mut drained = false;
        let mut max_up = 0.0f32;
        let mut min_y = f32::MAX;
        let mut time_up = 0u32; // consecutive frames not drained

        for frame in 0..(15.0 / DT) as u32 {
            let (la, ra) = match flip_mode {
                m if m < 0.4 => (frame % 80 < 30, frame % 80 < 30),
                m if m < 0.7 => (true, true),
                _ => (false, false),
            };
            if let Some(b) = world.bodies.iter().find(|b| b.borrow().id == left_id) {
                let mut b = b.borrow_mut();
                drive_flipper(
                    &mut b,
                    if la { -0.4 + 0.9 } else { -0.4 },
                    Vec2::new(FLIPPER_LENGTH * 0.45, 0.0),
                );
            }
            if let Some(b) = world.bodies.iter().find(|b| b.borrow().id == right_id) {
                let mut b = b.borrow_mut();
                drive_flipper(
                    &mut b,
                    if ra { 0.4 - 0.9 } else { 0.4 },
                    Vec2::new(-FLIPPER_LENGTH * 0.45, 0.0),
                );
            }
            let _ = world.step(DT);
            let (sp, bpos) = {
                let b = world.bodies.iter().find(|b| b.borrow().id == bid).unwrap();
                let b = b.borrow();
                (speed(&b), b.position)
            };
            max_up = max_up.max(bpos.y);
            min_y = min_y.min(bpos.y);
            if bpos.y < -3.0 {
                drained = true;
                break;
            }
            if sp < 0.4 {
                time_up += 1;
            } else {
                time_up = 0;
            }
            let _ = min_y;
        }

        total += 1;
        if !drained {
            stuck += 1;
            samples.push((x, y, max_up));
        } else {
            // even if drained, track if it got stuck long before
            let _ = ();
        }
        let _ = time_up;
    }

    println!("total={total} stuck(never_drained)={stuck}");
    for (x, y, my) in samples.iter().take(30) {
        println!("  STUCK drop=({x:.2},{y:.2}) max_up={my:.2}");
    }

    // No ball dropped onto the flippers may remain trapped in a wall/flipper
    // that would otherwise never drain.
    assert_eq!(
        stuck, 0,
        "{stuck} balls never drained (wedged in wall/flipper)"
    );
}
