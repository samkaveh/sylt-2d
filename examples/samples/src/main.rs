use nannou::prelude::*;
use nannou_egui::{self, egui, Egui};
use sylt_2d::body::{Body, ConvexPolygon, Shape};
use sylt_2d::joint::Joint;
use sylt_2d::log::Logger;
use sylt_2d::math_utils::{Mat2x2, Vec2};
use sylt_2d::metaball::{cluster_metaballs, compute_metaball_bounds, marching_squares_debug, Metaball};
use sylt_2d::world::World;
fn main() {
    nannou::app(model).update(update).run();
}
const ITERATIONS: u32 = 100;

struct EguiSettings {
    scale: f32,
    color: Srgb<u8>,
    metaball_threshold: f32,
    metaball_resolution: usize,
    metaball_strength: f32,
    metaball_radius: f32,
    metaball_cluster_threshold: f32,
    debug_show_grid: bool,
    debug_show_heatmap: bool,
    debug_show_segments: bool,
    debug_show_polygons: bool,
    debug_show_open_chains: bool,
    debug_show_case_labels: bool,
}

struct Model {
    _window: window::Id,
    time_step: f32,
    demo_index: u32,
    world: World,
    bomb: bool,
    egui: Egui,
    settings: EguiSettings,
    is_first_frame: bool,
    load_demo_flag: bool,
    metaball_bodies: Vec<usize>,
    logging_enabled: bool,
}

fn model(app: &App) -> Model {
    let _window = app
        .new_window()
        .view(view)
        .raw_event(raw_window_event)
        .key_pressed(key_pressed)
        .build()
        .unwrap();
    let window = app.window(_window).unwrap();
    let egui = Egui::from_window(&window);
    let world = World::new(Vec2::new(0.0, -10.0), ITERATIONS);
    Model {
        _window,
        world,
        demo_index: 0,
        bomb: false,
        time_step: 1.0 / 60.0,
        egui,
        settings: EguiSettings {
            scale: 18.0,
            color: WHITE,
            metaball_threshold: 0.5,
            metaball_resolution: 50,
            metaball_strength: 1.0,
            metaball_radius: 1.0,
            metaball_cluster_threshold: 0.5,
            debug_show_grid: false,
            debug_show_heatmap: false,
            debug_show_segments: false,
            debug_show_polygons: false,
            debug_show_open_chains: false,
            debug_show_case_labels: false,
        },
        is_first_frame: true,
        load_demo_flag: false,
        metaball_bodies: Vec::new(),
        logging_enabled: false,
    }
}

fn launch_bomb(model: &mut Model) {
    let mut bomb = Body::new(Vec2::new(1.0, 1.0), 50.0);
    bomb.friction = 0.2;
    bomb.position = Vec2::new(random_range(-15.0, 15.0), 15.0);
    bomb.rotation = random_range(-1.5, 1.5);
    bomb.velocity = bomb.position * -1.5;
    bomb.angular_velocity = random_range(-20.0, 20.0);
    model.world.add_body(bomb.clone());
}

fn demo1(_model: &mut Model) {
    // Single Shapes Falling
    let mut body1 = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    body1.position = Vec2::new(0.0, -0.5 * body1.width.y);
    _model.world.add_body(body1.clone());

    let mut body2 = Body::new(Vec2::new(1.0, 1.0), 200.0);
    body2.position = Vec2::new(0.0, 3.0);
    _model.world.add_body(body2.clone());

    // polygon: A hexagon
    let hexagon: Vec<Vec2> = vec![
        Vec2 { x: 0.0, y: 1.0 },    // Top vertex
        Vec2 { x: -0.87, y: 0.5 },  // Top-left vertex
        Vec2 { x: -0.87, y: -0.5 }, // Bottom-left vertex
        Vec2 { x: 0.0, y: -1.0 },   // Bottom vertex
        Vec2 { x: 0.87, y: -0.5 },  // Bottom-right vertex
        Vec2 { x: 0.87, y: 0.5 },   // Top-right vertex
    ];
    // polygon: A pentagon
    let pentagon: Vec<Vec2> = vec![
        Vec2 { x: 0.0, y: 1.0 },     // Top vertex
        Vec2 { x: -0.95, y: 0.31 },  // Top-left vertex
        Vec2 { x: -0.59, y: -0.81 }, // Bottom-left vertex
        Vec2 { x: 0.59, y: -0.81 },  // Bottom-right vertex
        Vec2 { x: 0.95, y: 0.31 },   // Top-right vertex
    ];
    let mut pentagon_body = Body::new_polygon(pentagon, 2.0);
    let mut hexagon_body = Body::new_polygon(hexagon, 2.0);
    pentagon_body.position = Vec2::new(0.0, 5.0);
    pentagon_body.friction = 100.0;
    hexagon_body.position = Vec2::new(5.0, 4.0);
    hexagon_body.rotation = 45.0;
    _model.world.add_body(pentagon_body.clone());
    _model.world.add_body(hexagon_body.clone());
}

fn demo2(model: &mut Model) {
    // Simple Pendulum
    let mut body1 = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    body1.friction = 0.2;
    body1.position = Vec2::new(0.0, -0.5 * body1.width.y);
    body1.rotation = 0.0;
    model.world.add_body(body1.clone());

    let mut body2 = Body::new(Vec2::new(1.0, 1.0), 100.0);
    body2.friction = 0.2;
    body2.position = Vec2::new(9.0, 11.0);
    body2.rotation = 0.0;
    model.world.add_body(body2.clone());

    let joint = Joint::new(body1, body2, Vec2::new(0.0, 11.0), &model.world);
    model.world.add_joint(joint);
}

fn demo3(model: &mut Model) {
    let friction_values = [0.75, 0.5, 0.35, 0.1, 0.0];

    let mut body = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    body.position = Vec2::new(0.0, -0.5 * body.width.y);
    model.world.add_body(body.clone());

    let mut body2 = Body::new(Vec2::new(13.0, 0.25), f32::MAX);
    body2.position = Vec2::new(-2.0, 11.0);
    body2.rotation = -0.25;
    model.world.add_body(body2.clone());

    // Additional bodies with varying frictions
    for (i, &friction) in friction_values.iter().enumerate() {
        let mut body = Body::new(Vec2::new(0.5, 0.5), 25.0);
        body.friction = friction;
        body.position = Vec2::new(-7.5 + 2.0 * i as f32, 14.0);
        model.world.add_body(body.clone());
    }
}

fn demo4(model: &mut Model) {
    // Vertical Stack
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.2;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    model.world.add_body(ground.clone());

    for i in 0..10 {
        let mut body = Body::new(Vec2::new(1.0, 1.0), 1.0);
        body.friction = 0.2;
        body.position = Vec2::new(random::<f32>() * 0.2 - 0.1, 0.51 + 1.05 * i as f32);
        model.world.add_body(body.clone());
    }
}

fn demo5(model: &mut Model) {
    // Pyramid
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.2;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    model.world.add_body(ground.clone());

    let mut x = Vec2::new(-6.0, 0.75);
    for i in 0..12 {
        let mut y = x;
        for _j in i..12 {
            let mut body = Body::new(Vec2::new(1.0, 1.0), 10.0);
            body.friction = 0.2;
            body.position = y;
            model.world.add_body(body.clone());

            y.x += 1.125;
        }
        x.x += 0.5625;
        x.y += 2.0;
    }
}

fn demo6(model: &mut Model) {
    // A Teeter
    let mut body1 = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    body1.position = Vec2::new(0.0, -0.5 * body1.width.y);
    model.world.add_body(body1.clone());

    let mut body2 = Body::new(Vec2::new(12.0, 0.25), 10.0);
    body2.position = Vec2::new(0.0, 3.0);
    model.world.add_body(body2.clone());

    let mut body3 = Body::new(Vec2::new(0.5, 0.5), 2.0);
    body3.position = Vec2::new(-5.0, 5.0);
    model.world.add_body(body3.clone());

    let mut body4 = Body::new(Vec2::new(0.5, 0.5), 2.0);
    body4.position = Vec2::new(-5.5, 5.0);
    model.world.add_body(body4.clone());

    let mut body5 = Body::new(Vec2::new(1.0, 1.0), 55.0);
    body5.position = Vec2::new(5.5, 15.0);
    model.world.add_body(body5.clone());

    let joint = Joint::new(body1, body2, Vec2::new(0.0, 3.0), &model.world);
    model.world.add_joint(joint);
}

fn demo7(model: &mut Model) {
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.2;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    model.world.add_body(ground.clone());

    let num_planks = 15;
    let mass = 10.0;
    let frequency_hz = 2.0;
    let damping_ratio = 0.7;
    let omega = 2.0 * std::f32::consts::PI * frequency_hz;
    let d = 2.0 * mass * damping_ratio * omega;
    let k = mass * omega * omega;
    let time_step = 1.0 / 60.0;
    let softness = 1.0 / (d + time_step * k);
    let bias_factor = time_step * k / (d + time_step * k);

    for i in 0..=num_planks {
        let mut plank = Body::new(Vec2::new(1.0, 0.25), mass);
        plank.friction = 0.2;
        plank.position = Vec2::new(-8.5 + 1.25 * i as f32, 5.0);
        model.world.add_body(plank.clone());

        let mut joint = Joint::new(
            plank.clone(),
            ground.clone(),
            Vec2::new(-9.125 + 1.25 * i as f32, 5.0),
            &model.world,
        );
        joint.softness = softness;
        joint.bias_factor = bias_factor;
        model.world.add_joint(joint);
    }
}

// Dominos demo
fn demo8(model: &mut Model) {
    let mut b1 = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    b1.position = Vec2::new(0.0, -0.5 * b1.width.y);
    model.world.add_body(b1.clone());

    let mut b = Body::new(Vec2::new(12.0, 0.5), f32::MAX);
    b.position = Vec2::new(-1.5, 10.0);
    model.world.add_body(b.clone());

    for i in 0..10 {
        let mut domino = Body::new(Vec2::new(0.2, 2.0), 10.0);
        domino.position = Vec2::new(-6.0 + 1.0 * i as f32, 11.125);
        domino.friction = 0.1;
        model.world.add_body(domino.clone());
    }

    let mut bb = Body::new(Vec2::new(14.0, 0.5), f32::MAX);
    bb.position = Vec2::new(1.0, 6.0);
    bb.rotation = 0.3;
    model.world.add_body(bb.clone());

    let mut b2 = Body::new(Vec2::new(0.5, 3.0), f32::MAX);
    b2.position = Vec2::new(-7.0, 4.0);
    model.world.add_body(b2.clone());

    let mut b3 = Body::new(Vec2::new(12.0, 0.25), 10.0);
    b3.position = Vec2::new(-0.9, 1.0);
    model.world.add_body(b3.clone());

    let joint1 = Joint::new(b1.clone(), b3, Vec2::new(-2.0, 3.0), &model.world);
    model.world.add_joint(joint1);

    let mut b4 = Body::new(Vec2::new(0.5, 0.5), 16.0);
    b4.position = Vec2::new(-10.0, 15.0);
    b4.rotation = 0.0;
    b4.friction = 0.2;
    model.world.add_body(b4.clone());

    let joint2 = Joint::new(b2, b4, Vec2::new(-7.0, 15.0), &model.world);
    model.world.add_joint(joint2);

    let mut b5 = Body::new(Vec2::new(2.0, 2.0), 10.0);
    b5.position = Vec2::new(6.0, 2.5);
    b5.friction = 0.1;
    model.world.add_body(b5.clone());

    let joint3 = Joint::new(b1, b5.clone(), Vec2::new(6.0, 2.6), &model.world);
    model.world.add_joint(joint3);

    let mut b6 = Body::new(Vec2::new(2.0, 0.2), 10.0);
    b6.position = Vec2::new(6.0, 3.6);
    model.world.add_body(b6.clone());

    let joint4 = Joint::new(b5, b6, Vec2::new(7.0, 3.5), &model.world);
    model.world.add_joint(joint4);
}

// Multi-pendulum demo
fn demo9(model: &mut Model) {
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.2;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    ground.rotation = 0.0;
    model.world.add_body(ground.clone());

    let mut b1 = ground;
    let mass = 10.0;
    let frequency_hz = 4.0;
    let damping_ratio = 0.7;

    let omega = 2.0 * std::f32::consts::PI * frequency_hz;
    let d = 2.0 * mass * damping_ratio * omega;
    let k = mass * omega * omega;

    let time_step = model.time_step;
    let softness = 1.0 / (d + time_step * k);
    let bias_factor = time_step * k / (d + time_step * k);

    let y = 12.0;

    for i in 0..15 {
        let mut pendulum = Body::new(Vec2::new(0.75, 0.25), mass);
        pendulum.friction = 0.2;
        pendulum.position = Vec2::new(0.5 + i as f32, y);
        pendulum.rotation = 0.0;
        model.world.add_body(pendulum.clone());

        let mut joint = Joint::new(b1, pendulum.clone(), Vec2::new(i as f32, y), &model.world);
        joint.softness = softness;
        joint.bias_factor = bias_factor;
        model.world.add_joint(joint);

        b1 = pendulum;
    }
}

// A Pawn and the pendulum
fn demo10(_model: &mut Model) {
    // A Pawn and the pendulum
    let pentagon: Vec<Vec2> = vec![
        Vec2 { x: 0.0, y: 1.0 },     // Top vertex
        Vec2 { x: -0.95, y: 0.31 },  // Top-left vertex
        Vec2 { x: -0.59, y: -0.81 }, // Bottom-left vertex
        Vec2 { x: 0.59, y: -0.81 },  // Bottom-right vertex
        Vec2 { x: 0.95, y: 0.31 },   // Top-right vertex
    ];
    let mut pawn_head = ConvexPolygon::new(vec![
        Vec2 { x: 0.0, y: 0.4 },   // Top center (head)
        Vec2 { x: -0.4, y: 0.0 },  // Upper left
        Vec2 { x: -0.2, y: -0.4 }, // Lower left (neck)
        Vec2 { x: 0.2, y: -0.4 },  // Lower right (neck)
        Vec2 { x: 0.4, y: 0.0 },   // Upper right
    ]);
    pawn_head.scale(2.0);

    // Define the bottom part (body and base) as a convex polygon
    let mut pawn_trunk = ConvexPolygon::new(vec![
        Vec2 { x: -0.8, y: 0.8 },  // Base left
        Vec2 { x: -0.6, y: 0.0 },  // Mid-body left
        Vec2 { x: -0.3, y: -0.8 }, // Bottom left
        Vec2 { x: 0.3, y: -0.8 },  // Bottom right
        Vec2 { x: 0.6, y: 0.0 },   // Mid-body right
        Vec2 { x: 0.8, y: 0.8 },   // Base right
    ]);
    pawn_trunk.scale(2.0);
    let mut body1 = Body::new(Vec2::new(1000.0, 20.0), f32::MAX);
    body1.position = Vec2::new(0.0, -0.5 * body1.width.y);
    _model.world.add_body(body1.clone());

    let mut pentagon_body = Body::new_polygon(pentagon, 55.0);
    let mut pawn_head = Body::new_polygon(pawn_head.get_vertices(), 10.0);
    let mut pawn_body = Body::new_polygon(pawn_trunk.get_vertices(), 10.0);
    pentagon_body.friction = 0.2;
    pentagon_body.position = Vec2::new(-9.0, 8.0);
    pentagon_body.rotation = 0.0;

    pawn_body.position = Vec2::new(5.0, 4.0);
    pawn_head.position = Vec2::new(
        pawn_body.position.x,
        pawn_body.position.y + 0.5 * pawn_body.width.y + 0.5 * pawn_head.width.y,
    );
    _model.world.add_body(pentagon_body.clone());
    _model.world.add_body(pawn_head.clone());
    _model.world.add_body(pawn_body.clone());
    let joint3 = Joint::new(pawn_head, pawn_body, Vec2::new(5.0, 3.0), &_model.world);
    _model.world.add_joint(joint3);

    let joint = Joint::new(body1, pentagon_body, Vec2::new(0.0, 11.0), &_model.world);
    _model.world.add_joint(joint);
}

fn demo11(model: &mut Model) {
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.2;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    model.world.add_body(ground.clone());

    model.metaball_bodies.clear();

    let radius = model.settings.metaball_radius;
    let spacing = radius * 2.5;
    for i in 0..6 {
        let mut body = Body::new_circle(radius, 1.0);
        body.friction = 0.3;
        body.position = Vec2::new(-spacing * 2.5 + i as f32 * spacing, 8.0 + i as f32 * 0.5);
        model.metaball_bodies.push(body.id);
        model.world.add_body(body);
    }
}

fn demo12(model: &mut Model) {
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.2;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    model.world.add_body(ground.clone());

    model.metaball_bodies.clear();

    let radius = model.settings.metaball_radius;
    let mut body1 = Body::new_circle(radius, 1.0);
    body1.friction = 0.3;
    body1.position = Vec2::new(-1.5, 3.0);
    model.metaball_bodies.push(body1.id);
    model.world.add_body(body1);

    let mut body2 = Body::new_circle(radius, 1.0);
    body2.friction = 0.3;
    body2.position = Vec2::new(1.5, 3.0);
    model.metaball_bodies.push(body2.id);
    model.world.add_body(body2);
}

fn demo13(model: &mut Model) {
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.2;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    model.world.add_body(ground.clone());

    model.metaball_bodies.clear();

    let radius = model.settings.metaball_radius;
    let mut body1 = Body::new_circle(radius, 1.0);
    body1.friction = 0.3;
    body1.position = Vec2::new(-1.5, 5.0);
    model.metaball_bodies.push(body1.id);
    model.world.add_body(body1);

    let mut body2 = Body::new_circle(radius, 1.0);
    body2.friction = 0.3;
    body2.position = Vec2::new(1.5, 5.0);
    model.metaball_bodies.push(body2.id);
    model.world.add_body(body2);
}

fn demo14(model: &mut Model) {
    let mut ground = Body::new(Vec2::new(100.0, 20.0), f32::MAX);
    ground.friction = 0.4;
    ground.position = Vec2::new(0.0, -0.5 * ground.width.y);
    model.world.add_body(ground);

    model.metaball_bodies.clear();

    let mut b1 = Body::new(Vec2::new(1.5, 1.5), 50.0);
    b1.position = Vec2::new(-4.0, 4.0);
    b1.friction = 0.5;
    model.world.add_body(b1);

    let mut b2 = Body::new(Vec2::new(2.0, 1.0), 80.0);
    b2.position = Vec2::new(0.0, 6.0);
    b2.rotation = 30.0_f32.to_radians();
    b2.friction = 0.3;
    model.world.add_body(b2);

    let mut b3 = Body::new(Vec2::new(1.0, 2.5), 100.0);
    b3.position = Vec2::new(4.0, 5.0);
    b3.rotation = -15.0_f32.to_radians();
    b3.friction = 0.2;
    model.world.add_body(b3);

    let mut c1 = Body::new_circle(0.6, 10.0);
    c1.position = Vec2::new(-2.5, 8.0);
    c1.friction = 0.4;
    model.world.add_body(c1);

    let mut c2 = Body::new_circle(0.8, 15.0);
    c2.position = Vec2::new(1.5, 9.0);
    c2.friction = 0.3;
    model.world.add_body(c2);

    let mut c3 = Body::new_circle(0.5, 8.0);
    c3.position = Vec2::new(-1.0, 10.0);
    c3.friction = 0.5;
    model.world.add_body(c3);

    let mut c4 = Body::new_circle(1.0, 20.0);
    c4.position = Vec2::new(3.0, 7.5);
    c4.friction = 0.2;
    model.world.add_body(c4);

    let pentagon: Vec<Vec2> = vec![
        Vec2 { x: 0.0, y: 0.9 },
        Vec2 { x: -0.86, y: 0.28 },
        Vec2 { x: -0.53, y: -0.73 },
        Vec2 { x: 0.53, y: -0.73 },
        Vec2 { x: 0.86, y: 0.28 },
    ];
    let mut p1 = Body::new_polygon(pentagon, 12.0);
    p1.position = Vec2::new(-3.5, 10.0);
    p1.friction = 0.6;
    model.world.add_body(p1);

    let hexagon: Vec<Vec2> = vec![
        Vec2 { x: 0.0, y: 0.8 },
        Vec2 { x: -0.69, y: 0.4 },
        Vec2 { x: -0.69, y: -0.4 },
        Vec2 { x: 0.0, y: -0.8 },
        Vec2 { x: 0.69, y: -0.4 },
        Vec2 { x: 0.69, y: 0.4 },
    ];
    let mut h1 = Body::new_polygon(hexagon, 15.0);
    h1.position = Vec2::new(2.0, 11.0);
    h1.friction = 0.4;
    model.world.add_body(h1);

    let triangle: Vec<Vec2> = vec![
        Vec2 { x: 0.0, y: 0.8 },
        Vec2 { x: -0.7, y: -0.5 },
        Vec2 { x: 0.7, y: -0.5 },
    ];
    let mut t1 = Body::new_polygon(triangle, 10.0);
    t1.position = Vec2::new(-1.5, 12.0);
    t1.rotation = 20.0_f32.to_radians();
    t1.friction = 0.5;
    model.world.add_body(t1);

    let mut wall_l = Body::new(Vec2::new(0.5, 6.0), f32::MAX);
    wall_l.position = Vec2::new(-8.0, 3.0);
    wall_l.friction = 0.1;
    model.world.add_body(wall_l);

    let mut wall_r = Body::new(Vec2::new(0.5, 6.0), f32::MAX);
    wall_r.position = Vec2::new(8.0, 3.0);
    wall_r.friction = 0.1;
    model.world.add_body(wall_r);
}

fn update(_app: &App, _model: &mut Model, _update: Update) {
    if _model.is_first_frame {
        let step = _model.world.step(_model.time_step);
        if let Err(e) = step {
            eprintln!("Error: {}", e);
        }
        // Load the initial demo
        load_demo(_model);
        _model.is_first_frame = false;
    }
    let step = _model.world.step(_model.time_step);
    if let Err(e) = step {
        eprintln!("Error: {}", e);
    }

    if _model.load_demo_flag {
        load_demo(_model);
        _model.load_demo_flag = false;
    }

    if _model.bomb {
        launch_bomb(_model);
        _model.bomb = false;
    }

    let egui = &mut _model.egui;
    let settings = &mut _model.settings;

    egui.set_elapsed_time(_update.since_start);
    let ctx = egui.begin_frame();
    let demo_names = [
        "Demo 1: Simple Shapes Falling",
        "Demo 2: Simple Pendulum",
        "Demo 3: Varying Friction Coefficients",
        "Demo 4: Randomized Stacking",
        "Demo 5: Pyramid Stacking",
        "Demo 6: A Teeter",
        "Demo 7: A Suspension Bridge",
        "Demo 8: Dominos",
        "Demo 9: Multi-pendulum",
        "Demo 10: A Pawn and the pendulum",
        "Demo 11: Metaballs",
        "Demo 12: 2 Metaballs (Static)",
        "Demo 13: 2 Metaballs (Physics)",
        "Demo 14: Mixed Shapes",
    ];
    egui::Window::new("Settings").show(&ctx, |ui| {
        // Dropdown for selecting the demo
        ui.label("Select Demo:");
        egui::ComboBox::from_label("Demo Selection")
            .selected_text(format!("Demo {}", _model.demo_index + 1))
            .show_ui(ui, |ui| {
                for (i, name) in demo_names.iter().enumerate() {
                    ui.selectable_value(&mut _model.demo_index, i as u32, *name);
                }
            });

        // Button to load the selected demo
        if ui.button("Load Demo").clicked() {
            _model.load_demo_flag = true;
        }
        // Scale slider
        ui.label("Scale:");
        ui.add(egui::Slider::new(&mut settings.scale, 0.0..=1000.0));

        // Random color button
        let clicked = ui.button("Random color").clicked();

        if clicked {
            settings.color = rgb(random(), random(), random());
        }

        if ui.button("launch bomb").clicked() {
            _model.bomb = true;
        }

        // Checkbox to enable a feature
        ui.checkbox(
            &mut _model.world.world_context.warm_starting,
            "Enable/Disable warm starting.",
        );
        ui.checkbox(
            &mut _model.world.world_context.position_correction,
            "Enable/Disable position correction.",
        );
        ui.checkbox(
            &mut _model.world.world_context.accumulate_impulse,
            "Enable/Disable accumulation of impulse.",
        );

        if _model.demo_index >= 10 {
            ui.separator();
            ui.label("Metaball Settings:");
            ui.add(
                egui::Slider::new(&mut settings.metaball_threshold, 0.1..=2.0).text("Threshold"),
            );
            ui.add(
                egui::Slider::new(&mut settings.metaball_resolution, 20..=100).text("Resolution"),
            );
            ui.add(egui::Slider::new(&mut settings.metaball_strength, 0.1..=5.0).text("Strength"));
            ui.add(egui::Slider::new(&mut settings.metaball_radius, 0.3..=2.0).text("Radius"));
            ui.add(
                egui::Slider::new(&mut settings.metaball_cluster_threshold, 0.0..=5.0).text("Cluster Threshold"),
            );
            ui.separator();
            ui.label("Debug Overlays:");
            ui.checkbox(&mut settings.debug_show_grid, "Show Grid");
            ui.checkbox(&mut settings.debug_show_heatmap, "Show Heatmap");
            ui.checkbox(&mut settings.debug_show_segments, "Show Segments");
            ui.checkbox(&mut settings.debug_show_polygons, "Show Polygons");
            ui.checkbox(&mut settings.debug_show_open_chains, "Show Open Chains");
            ui.checkbox(&mut settings.debug_show_case_labels, "Show Case Labels");
        }

        ui.separator();
        let was_logging = _model.logging_enabled;
        ui.checkbox(&mut _model.logging_enabled, "Enable JSON logging to file");
        if _model.logging_enabled && !was_logging {
            let logger = Logger::new("sylt2d_log.jsonl");
            _model.world.set_logger(logger);
            println!("JSON logging enabled -> sylt2d_log.jsonl");
        }
        if !_model.logging_enabled && was_logging {
            _model.world.logger = None;
            println!("JSON logging disabled");
        }
    });
}

fn load_demo(model: &mut Model) {
    model.world.clear(); // Clear the current world bodies and joints

    match model.demo_index {
        0 => demo1(model),
        1 => demo2(model),
        2 => demo3(model),
        3 => demo4(model),
        4 => demo5(model),
        5 => demo6(model),
        6 => demo7(model),
        7 => demo8(model),
        8 => demo9(model),
        9 => demo10(model),
        10 => demo11(model),
        11 => demo12(model),
        12 => demo13(model),
        13 => demo14(model),
        _ => {}
    }
}
fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    // Let egui handle things like keyboard and mouse input.
    model.egui.handle_raw_event(event);
}

fn key_pressed(_app: &App, model: &mut Model, key: Key) {
    match key {
        Key::Right => {
            let _ = model.world.step(model.time_step);
        }
        Key::Left => {
            let _ = model.world.step(-model.time_step);
        }
        Key::Return => {
            println!("Number of bodies {:?}", model.world.bodies.len());
            println!("World Bodies: {:?}", model.world.bodies);
            println!("{:?}", model.world.arbiters);
        }
        _other_key => {}
    }
}

fn view(app: &App, _model: &Model, frame: Frame) {
    let draw = app.draw();
    let draw = draw.scale(_model.settings.scale);
    let settings = &_model.settings;
    draw.background().color(SLATEGREY);

    for (num, body) in _model.world.iter_bodies().enumerate() {
        match body.shape {
            Shape::Box => {
                draw.rect()
                    .x_y(body.position.x, body.position.y)
                    .w_h(body.width.x, body.width.y)
                    .rotate(body.rotation)
                    .color(if num == 0 { DARKSEAGREEN } else { ORCHID });
            }
            Shape::ConvexPolygon => {
                let tuples: Vec<(f32, f32)> = body
                    .get_polygon()
                    .get_vertices()
                    .into_iter()
                    .map(Into::into)
                    .collect();
                draw.polygon()
                    .color(if num == 0 { DARKSEAGREEN } else { ORCHID })
                    .x_y(body.position.x, body.position.y)
                    .rotate(body.rotation)
                    .points(tuples);
            }
            Shape::Circle => {
                if _model.demo_index == 12 {
                    // Skip individual circle rendering; metaball contour replaces it
                } else {
                    draw.ellipse()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.radius * 2.0, body.radius * 2.0)
                        .color(if num == 0 { DARKSEAGREEN } else { ORCHID });
                }
            }
        }
    }

    if _model.demo_index >= 10 {
        let metaballs: Vec<Metaball> = _model
            .world
            .iter_bodies()
            .filter(|b| b.shape == Shape::Circle && _model.metaball_bodies.contains(&b.id))
            .map(|b| {
                Metaball::new(
                    Vec2::new(b.position.x, b.position.y),
                    b.radius * 2.0,
                    settings.metaball_strength,
                )
            })
            .collect();

        if !metaballs.is_empty() {
            let clusters = cluster_metaballs(&metaballs, settings.metaball_cluster_threshold);

            for cluster in &clusters {
                let (bounds_min, bounds_max) =
                    compute_metaball_bounds(&cluster.metaballs, settings.metaball_threshold, 3.0);

                let debug = marching_squares_debug(
                    &metaballs,
                    bounds_min,
                    bounds_max,
                    settings.metaball_resolution,
                    settings.metaball_threshold,
                );

                if settings.debug_show_heatmap {
                    let res = debug.resolution;
                    for j in 0..res {
                        for i in 0..res {
                            let v = debug.grid[j][i];
                            let t =
                                ((v - debug.threshold) / debug.threshold.max(0.01)).clamp(-1.0, 1.0);
                            let r = if t > 0.0 { t } else { 0.0 };
                            let b = if t < 0.0 { -t } else { 0.0 };
                            let g = 0.2;
                            let x = debug.bounds_min.x + (i as f32 + 0.5) * debug.cell_w;
                            let y = debug.bounds_min.y + (j as f32 + 0.5) * debug.cell_h;
                            draw.rect()
                                .x_y(x, y)
                                .w_h(debug.cell_w, debug.cell_h)
                                .color(rgba(r, g, b, 0.4));
                        }
                    }
                }

                if settings.debug_show_grid {
                    let res = debug.resolution;
                    for i in 0..=res {
                        let x = debug.bounds_min.x + i as f32 * debug.cell_w;
                        draw.line()
                            .start(pt2(x, debug.bounds_min.y))
                            .end(pt2(x, debug.bounds_max.y))
                            .weight(0.02)
                            .color(rgba(1.0, 1.0, 1.0, 0.3));
                    }
                    for j in 0..=res {
                        let y = debug.bounds_min.y + j as f32 * debug.cell_h;
                        draw.line()
                            .start(pt2(debug.bounds_min.x, y))
                            .end(pt2(debug.bounds_max.x, y))
                            .weight(0.02)
                            .color(rgba(1.0, 1.0, 1.0, 0.3));
                    }
                }

                if settings.debug_show_segments {
                    for seg in &debug.segments {
                        draw.line()
                            .start(pt2(seg.0.x, seg.0.y))
                            .end(pt2(seg.1.x, seg.1.y))
                            .weight(0.08)
                            .color(YELLOW);
                    }
                }

                if settings.debug_show_polygons {
                    for poly in &debug.polygons {
                        if poly.len() >= 3 {
                            draw.polygon()
                                .x_y(0.0, 0.0)
                                .color(rgba(0.2, 0.8, 0.4, 0.7))
                                .points(poly.clone());
                        }
                    }
                }

                if settings.debug_show_open_chains {
                    for chain in &debug.open_chains {
                        for w in chain.windows(2) {
                            draw.line()
                                .start(pt2(w[0].0, w[0].1))
                                .end(pt2(w[1].0, w[1].1))
                                .weight(0.1)
                                .color(RED);
                        }
                    }
                }

                if settings.debug_show_case_labels {
                    for cell in &debug.cells {
                        if cell.case_index != 0 && cell.case_index != 15 {
                            let x = cell.x0 + cell.cell_w * 0.5;
                            let y = cell.y0 + cell.cell_h * 0.5;
                            draw.text(&cell.case_index.to_string())
                                .x_y(x, y)
                                .color(WHITE)
                                .font_size(8);
                        }
                    }
                }

                if !settings.debug_show_polygons && !settings.debug_show_segments {
                    for poly in &debug.polygons {
                        if poly.len() >= 3 {
                            draw.polygon()
                                .x_y(0.0, 0.0)
                                .color(rgba(0.2, 0.8, 0.4, 0.7))
                                .points(poly.clone());
                        }
                    }
                }
            }
        }
    }

    for (_, arbiter) in _model.world.arbiters.iter() {
        for contact in arbiter.contacts.iter() {
            match contact {
                Some(contact) => {
                    draw.ellipse()
                        .x_y(contact.position.x, contact.position.y)
                        .radius(0.1)
                        .color(settings.color);
                    draw.arrow()
                        .start(pt2(contact.position.x, contact.position.y))
                        .end(pt2(
                            contact.position.x + contact.normal.x,
                            contact.position.y + contact.normal.y,
                        ))
                        .weight(0.05)
                        .color(LIGHTSALMON);
                }
                None => (),
            }
        }
    }
    for joint in _model.world.joints.iter() {
        let x1 = joint.body_1.borrow().position;
        let x2 = joint.body_2.borrow().position;
        let r1 = Mat2x2::new_from_angle(joint.body_1.borrow().rotation);
        let r2 = Mat2x2::new_from_angle(joint.body_2.borrow().rotation);
        let p1 = x1 + r1 * joint.local_anchor_1;
        let p2 = x2 + r2 * joint.local_anchor_2;
        draw.line()
            .start(pt2(x1.x, x1.y))
            .end(pt2(p1.x, p1.y))
            .weight(0.05)
            .color(SLATEBLUE);
        draw.line()
            .start(pt2(x2.x, x2.y))
            .end(pt2(p2.x, p2.y))
            .weight(0.05)
            .color(SLATEBLUE);
    }
    draw.to_frame(app, &frame).unwrap();
    _model.egui.draw_to_frame(&frame).unwrap();
}
