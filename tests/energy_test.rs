use sylt_2d::body::Body;
use sylt_2d::math_utils::Vec2;
use sylt_2d::world::World;

#[test]
fn ball_bounces_with_restitution() {
    let mut world = World::new(Vec2::new(0.0, -10.0), 40);
    world.set_restitution(0.3);
    let mut ground = Body::new(Vec2::new(30.0, 1.0), f32::MAX);
    ground.position = Vec2::new(0.0, -5.0);
    world.add_body(ground);
    let mut ball = Body::new_circle(0.35, 1.0);
    ball.position = Vec2::new(0.0, 5.0);
    ball.velocity = Vec2::new(0.0, -5.0);
    let id = ball.id;
    world.add_body(ball);
    let dt = 1.0/60.0;
    let mut max_y = -100.0f32;
    for _ in 0..100 {
        let _ = world.step(dt);
        let b = world.bodies.iter().find(|b| b.borrow().id == id).unwrap();
        let py = b.borrow().position.y;
        if py > max_y { max_y = py; }
    }
    // Should have bounced back up (reached a height above the release after first bounce).
    println!("max_y {}", max_y);
    assert!(max_y > 4.0, "ball did not bounce back, max_y={}", max_y);
}
