use sylt_2d::body::Body;
use sylt_2d::math_utils::Vec2;
use sylt_2d::world::World;

#[test]
fn ccd_fast_box_does_not_fling_stack_or_tunnel() {
    // A fast-moving dynamic box should be resolved at the impact point and
    // must not fling the static ground or any other body off-screen.
    let mut world = World::new(Vec2::new(0.0, -10.0), 15);

    // Static ground.
    let mut ground = Body::new(Vec2::new(30.0, 1.0), f32::MAX);
    ground.position = Vec2::new(0.0, -5.0);
    world.add_body(ground);

    // A resting stack of boxes.
    for y in 0..4 {
        let mut b = Body::new(Vec2::new(2.0, 2.0), 2.0);
        b.position = Vec2::new(0.0, -3.0 + y as f32 * 2.0);
        world.add_body(b);
    }

    let ground_id = 1; // ground was added first

    // Fast-moving box dropped from above onto the stack.
    let mut fast = Body::new(Vec2::new(2.0, 2.0), 5.0);
    fast.position = Vec2::new(0.0, 12.0);
    fast.velocity = Vec2::new(0.0, -60.0); // well above threshold
    let fast_id = fast.id;
    world.add_body(fast);

    let dt = 1.0 / 60.0;
    for _ in 0..120 {
        let _ = world.step(dt);
    }

    // The ground must never have moved (static).
    let g = world
        .bodies
        .iter()
        .find(|b| b.borrow().id == ground_id)
        .unwrap();
    let gp = g.borrow().position;
    assert!(
        (gp.x - 0.0).abs() < 1e-3 && (gp.y - (-5.0)).abs() < 1e-3,
        "static ground moved! pos={:?}",
        gp
    );

    // The fast box should have settled near the stack, not flown off.
    let f = world
        .bodies
        .iter()
        .find(|b| b.borrow().id == fast_id)
        .unwrap();
    let fp = f.borrow().position;
    assert!(
        fp.y > -5.0 && fp.y < 5.0,
        "fast box flew off or tunneled, y={}",
        fp.y
    );
    assert!(
        fp.x.abs() < 20.0,
        "fast box flew sideways, x={}",
        fp.x
    );
}
