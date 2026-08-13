use sylt_2d::body::Body;
use sylt_2d::math_utils::Vec2;
use sylt_2d::world::World;

#[test]
fn fast_circle_does_not_tunnel_through_static_wall_with_ccd() {
    // A fast-moving ball should collide with a static wall, not pass through,
    // when CCD is enabled.
    let mut world = World::new(Vec2::new(0.0, 0.0), 10);

    // Static wall (inv_mass == 0).
    let mut wall = Body::new(Vec2::new(20.0, 1.0), f32::MAX);
    wall.position = Vec2::new(0.0, -5.0);
    world.add_body(wall);

    // Fast-moving dynamic circle heading straight into the wall.
    let mut ball = Body::new_circle(0.35, 1.0);
    ball.position = Vec2::new(0.0, 5.0);
    ball.velocity = Vec2::new(0.0, -120.0); // ~7.5 units/frame, well above threshold
    let ball_id = ball.id;
    world.add_body(ball);

    let dt = 1.0 / 60.0;
    // Step for several frames.
    for _ in 0..20 {
        let _ = world.step(dt);
    }

    // Ball should have been reflected and should not have passed through the wall.
    let b = world
        .bodies
        .iter()
        .find(|body| body.borrow().id == ball_id)
        .expect("ball still in world");
    let pos = b.borrow().position;
    // Wall top is at y = -5 - 0.5 = -5.5. Ball radius 0.35 so its center should
    // never be below -5.85 (would mean it tunneled).
    assert!(pos.y > -5.85, "ball tunneled through wall, y={}", pos.y);
}
