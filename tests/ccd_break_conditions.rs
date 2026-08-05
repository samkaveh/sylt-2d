use sylt_2d::body::{Body, Shape};
use sylt_2d::math_utils::Vec2;
use sylt_2d::world::World;

#[test]
fn ccd_arc_fast_circle_near_tangent() {
    // Ball approaching polygon arc at shallow angle (tangent-like).
    // Arc segment from the pinball cabinet: a convex polygon segment.
    let mut world = World::new(Vec2::new(0.0, -10.0), 15);
    world.set_ccd_enabled(true);

    // Approximate arc segment (small box-like polygon for simplicity).
    let arc = Body::new(Vec2::new(2.0, 0.5), f32::MAX);
    world.add_body(arc);

    let mut ball = Body::new_circle(0.35, 1.5);
    ball.position = Vec2::new(-1.5, 2.0);
    ball.velocity = Vec2::new(80.0, -40.0);
    world.add_body(ball);

    let dt = 1.0 / 60.0;
    for _ in 0..60 {
        world.step(dt).unwrap();
    }
}

#[test]
fn ccd_bridge_link_fast_impact() {
    // Ball falling onto a moving/soft bridge link.
    let mut world = World::new(Vec2::new(0.0, -10.0), 15);
    world.set_ccd_enabled(true);

    let link = Body::new(Vec2::new(0.3, 2.0), 1.0);
    world.add_body(link);

    let mut ball = Body::new_circle(0.35, 1.5);
    ball.position = Vec2::new(0.0, 5.0);
    ball.velocity = Vec2::new(0.0, -100.0);
    world.add_body(ball);

    let dt = 1.0 / 60.0;
    for _ in 0..30 {
        world.step(dt).unwrap();
    }
}

#[test]
fn ccd_arc_near_end_of_step_impact() {
    // Impact that would occur very close to t=1 (end of step).
    // This previously failed due to the 0.95 filter.
    let mut world = World::new(Vec2::new(0.0, 0.0), 10);
    world.set_ccd_enabled(true);

    // Small polygon segment representing part of the arc.
    let arc_seg = Body::new_polygon(
        vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(2.0, 0.5),
            Vec2::new(0.0, 0.5),
        ],
        f32::MAX,
    );
    world.add_body(arc_seg);

    let mut ball = Body::new_circle(0.35, 1.0);
    ball.position = Vec2::new(1.0, 2.0);
    ball.velocity = Vec2::new(0.0, -60.0);
    world.add_body(ball);

    let dt = 1.0 / 60.0;
    for _ in 0..20 {
        world.step(dt).unwrap();
    }
}
