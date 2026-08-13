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

fn build_single_flipper(world: &mut World) -> (usize, Vec2, f32) {
    let mut floor = Body::new(Vec2::new(20.0, 1.0), f32::MAX);
    floor.position = Vec2::new(0.0, -5.0);
    floor.friction = 0.2;
    world.add_body(floor);

    let mut lw = Body::new(Vec2::new(0.6, 26.0), f32::MAX);
    lw.position = Vec2::new(-8.3, 9.0);
    lw.friction = 0.05;
    world.add_body(lw);

    let pivot = Vec2::new(-3.2, -0.6);
    let rest = -0.4;
    let offset_x = FLIPPER_LENGTH * 0.45;
    let rot_mat = sylt_2d::math_utils::Mat2x2::new_from_angle(rest);
    let mut flipper = Body::new(Vec2::new(FLIPPER_LENGTH, FLIPPER_WIDTH), 15.0);
    flipper.position = pivot + rot_mat * Vec2::new(offset_x, 0.0);
    flipper.friction = 0.6;
    flipper.rotation = rest;
    let id = flipper.id;
    world.add_body(flipper.clone());

    let mut anchor = Body::new(Vec2::new(0.1, 0.1), f32::MAX);
    anchor.position = pivot;
    world.add_body(anchor.clone());
    let mut joint = Joint::new(anchor, flipper, pivot, world);
    joint.softness = 0.005;
    world.add_joint(joint);
    (id, pivot, rest)
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

#[test]
fn trace_stuck_case() {
    let mut world = World::new(Vec2::new(0.0, -18.0), 120);
    world.set_restitution(0.35);
    let (fid, _pivot, rest) = build_single_flipper(&mut world);

    let mut ball = Body::new_circle(BALL_RADIUS, 1.5);
    ball.friction = 0.02;
    ball.position = Vec2::new(-3.04, -0.65);
    let bid = ball.id;
    world.add_body(ball);

    for frame in 0..(6.0 / DT) as u32 {
        if let Some(b) = world.bodies.iter().find(|b| b.borrow().id == fid) {
            let mut b = b.borrow_mut();
            drive_flipper(&mut b, rest, Vec2::new(FLIPPER_LENGTH * 0.45, 0.0));
        }
        let _ = world.step(DT);

        // Inspect arbiters involving the ball.
        let mut lines = Vec::new();
        for (key, arb) in world.arbiters.iter() {
            let (aid, bid_) = (key.body1_id(), key.body2_id());
            if aid == bid || bid_ == bid {
                for c in arb.contacts.iter().flatten() {
                    let b1 = world.bodies.iter().find(|b| b.borrow().id == aid).unwrap();
                    let b2 = world.bodies.iter().find(|b| b.borrow().id == bid_).unwrap();
                    let (s1, s2) = (b1.borrow().shape, b2.borrow().shape);
                    lines.push(format!(
                        "arbiter({aid},{bid_}) shapes=({s1:?},{s2:?}) sep={:.3} n=({:+.2},{:+.2}) pos=({:.2},{:.2}) pn={:.2}",
                        c.separation, c.normal.x, c.normal.y, c.position.x, c.position.y, c.pn
                    ));
                }
            }
        }
        let (sp, bpos) = {
            let b = world.bodies.iter().find(|b| b.borrow().id == bid).unwrap();
            let b = b.borrow();
            (speed(&b), b.position)
        };
        if frame < 150 || frame % 40 == 0 {
            println!(
                "f={frame:3} ball=({:+.2},{:+.2}) sp={sp:.2}",
                bpos.x, bpos.y
            );
            for l in &lines {
                println!("   {l}");
            }
        }
        if bpos.y < -3.0 {
            println!("DRAINED at {frame}");
            break;
        }
    }
}
