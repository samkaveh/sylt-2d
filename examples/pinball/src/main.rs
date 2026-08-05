use nannou::prelude::*;
use nannou_egui::{self, egui, Egui};
use sylt_2d::body::{Body, Shape};
use sylt_2d::joint::Joint;
use sylt_2d::math_utils::Vec2;
use sylt_2d::metaball::{
    cluster_metaballs, compute_metaball_bounds, marching_squares_debug, Metaball,
};
use sylt_2d::world::World;

fn main() {
    nannou::app(model).update(update).run();
}

const ITERATIONS: u32 = 120;
const BALL_RADIUS: f32 = 0.35;
const FLIPPER_LENGTH: f32 = 2.2;
const FLIPPER_WIDTH: f32 = 0.45;
const FLIPPER_UP_DELTA: f32 = 0.9;
const PLUNGER_MAX_CHARGE: f32 = 45.0;
const GRID_SNAP: f32 = 0.25;

// Default Spawn Position inside the Plunger Lane
fn default_ball_spawn() -> Vec2 {
    Vec2::new(7.0, -1.0)
}

fn vec2_normalize(v: Vec2) -> Vec2 {
    let len = v.length();
    if len < f32::EPSILON {
        Vec2::new(0.0, 0.0)
    } else {
        v * (1.0 / len)
    }
}

fn rotate_vec(v: Vec2, angle: f32) -> Vec2 {
    Vec2::new(
        v.x * angle.cos() - v.y * angle.sin(),
        v.x * angle.sin() + v.y * angle.cos(),
    )
}

fn snap_value(v: f32, step: f32) -> f32 {
    if step <= 0.0 {
        v
    } else {
        (v / step).round() * step
    }
}

fn dist_point_segment(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let len_sq = ab.dot(ab);
    if len_sq < f32::EPSILON {
        return (p - a).length();
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    (p - (a + ab * t)).length()
}

fn element_hit(elem: &BoardElement, pos: Vec2) -> bool {
    match &elem.kind {
        ElementKind::Wall { width, height } | ElementKind::Target { width, height, .. } => {
            let local = rotate_vec(pos - elem.position, -elem.rotation);
            local.x.abs() < width * 0.5 + 0.3 && local.y.abs() < height * 0.5 + 0.3
        }
        ElementKind::Bumper { radius, .. } | ElementKind::FluidPool { radius, .. } => {
            (pos - elem.position).length() < *radius + 0.3
        }
        ElementKind::Flipper { side, length } => {
            let offset_x = match side {
                FlipperSide::Left => length * 0.45,
                FlipperSide::Right => -length * 0.45,
            };
            let tip = elem.position + rotate_vec(Vec2::new(offset_x, 0.0), elem.rotation);
            dist_point_segment(pos, elem.position, tip) < FLIPPER_WIDTH * 0.5 + 0.3
        }
        ElementKind::Drain { width } => {
            let local = rotate_vec(pos - elem.position, -elem.rotation);
            local.x.abs() < width * 0.5 + 0.3 && local.y.abs() < 0.45
        }
        _ => {
            let dx = (pos.x - elem.position.x).abs();
            let dy = (pos.y - elem.position.y).abs();
            dx < 1.5 && dy < 1.5
        }
    }
}

fn element_extent(elem: &BoardElement) -> f32 {
    match &elem.kind {
        ElementKind::Wall { width, height } => width.max(*height) * 0.5,
        ElementKind::Bumper { radius, .. } => *radius,
        ElementKind::Flipper { length, .. } => length * 0.5,
        ElementKind::Chain { total_length, .. } => total_length * 0.5,
        ElementKind::FluidPool { radius, .. } => *radius,
        ElementKind::SoftBridge { .. } => 1.0,
        ElementKind::Target { width, height, .. } => width.max(*height) * 0.5,
        ElementKind::Drain { width } => *width * 0.5,
        ElementKind::BallSpawn => BALL_RADIUS,
    }
}

fn rotation_handle_pos(elem: &BoardElement) -> Vec2 {
    let extent = element_extent(elem).max(0.4);
    elem.position + rotate_vec(Vec2::new(extent + 0.45, 0.0), elem.rotation)
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum EditTool {
    Select,
    Wall,
    Bumper,
    FlipperLeft,
    FlipperRight,
    Chain,
    FluidPool,
    SoftBridge,
    Target,
    Drain,
    BallSpawn,
    Delete,
}

impl EditTool {
    fn name(&self) -> &str {
        match self {
            EditTool::Select => "Select/Move",
            EditTool::Wall => "Wall",
            EditTool::Bumper => "Bumper",
            EditTool::FlipperLeft => "Flipper (L)",
            EditTool::FlipperRight => "Flipper (R)",
            EditTool::Chain => "Chain/Pendulum",
            EditTool::FluidPool => "Fluid Pool",
            EditTool::SoftBridge => "Soft Bridge",
            EditTool::Target => "Target",
            EditTool::Drain => "Drain Zone",
            EditTool::BallSpawn => "Ball Spawn",
            EditTool::Delete => "Delete",
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum FluidType {
    Water,
    Slime,
    Lava,
    Acid,
}

impl FluidType {
    fn name(&self) -> &str {
        match self {
            FluidType::Water => "Water",
            FluidType::Slime => "Slime",
            FluidType::Lava => "Lava",
            FluidType::Acid => "Acid",
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum GameMode {
    Editor,
    Play,
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum FlipperSide {
    Left,
    Right,
}

#[derive(Clone, Debug)]
enum ElementKind {
    Wall {
        width: f32,
        height: f32,
    },
    Bumper {
        radius: f32,
        boost: f32,
        score: u32,
    },
    Flipper {
        side: FlipperSide,
        length: f32,
    },
    Chain {
        link_count: usize,
        total_length: f32,
        end_mass: f32,
    },
    FluidPool {
        radius: f32,
        fluid: FluidType,
        viscosity: f32,
    },
    SoftBridge {
        end_x: f32,
        segments: usize,
        softness: f32,
    },
    Target {
        width: f32,
        height: f32,
        score: u32,
    },
    Drain {
        width: f32,
    },
    BallSpawn,
}

#[derive(Clone, Debug)]
struct BoardElement {
    id: usize,
    position: Vec2,
    rotation: f32,
    kind: ElementKind,
    color: [f32; 3],
}

struct EditorState {
    tool: EditTool,
    selected_id: Option<usize>,
    drag_start: Option<Vec2>,
    dragging: bool,
    rotating: bool,
    wall_width: f32,
    wall_height: f32,
    bumper_radius: f32,
    bumper_boost: f32,
    bumper_score: u32,
    chain_links: usize,
    chain_length: f32,
    chain_end_mass: f32,
    fluid_radius: f32,
    fluid_type: FluidType,
    fluid_viscosity: f32,
    bridge_segments: usize,
    bridge_softness: f32,
    target_width: f32,
    target_height: f32,
    target_score: u32,
    drain_width: f32,
    next_color_idx: usize,
}

struct PlayState {
    score: u32,
    balls_remaining: u32,
    ball_body_id: Option<usize>,
    flipper_left_id: Option<usize>,
    flipper_right_id: Option<usize>,
    flipper_left_rest: f32,
    flipper_left_up: f32,
    flipper_right_rest: f32,
    flipper_right_up: f32,
    plunger_body_id: Option<usize>,
    plunger_charge: f32,
    plunger_charging: bool,
    flipper_left_active: bool,
    flipper_right_active: bool,
    chain_bodies: Vec<usize>,
    chain_anchor_id: Option<usize>,
    metaball_bodies: Vec<usize>,
    fluid_drag_active: bool,
    game_over: bool,
    high_score: u32,
    bumper_flash_timers: Vec<(Vec2, f32)>, // (position, timer)
}

struct EguiSettings {
    scale: f32,
    cam_x: f32,
    cam_y: f32,
    show_grid: bool,
    snap_to_grid: bool,
    grid_size: f32,
    show_contacts: bool,
    metaball_resolution: usize,
    metaball_threshold: f32,
}

struct Model {
    _window: window::Id,
    time_step: f32,
    world: World,
    egui: Egui,
    settings: EguiSettings,
    mode: GameMode,
    elements: Vec<BoardElement>,
    editor: EditorState,
    play: PlayState,
    next_id: usize,
    board_name_input: String,
    load_board_flag: bool,
    save_board_flag: bool,
    mouse_world: Vec2,
    is_first_frame: bool,
}

fn model(app: &App) -> Model {
    let _window = app
        .new_window()
        .view(view)
        .raw_event(raw_window_event)
        .key_pressed(key_pressed)
        .key_released(key_released)
        .mouse_pressed(mouse_pressed)
        .mouse_released(mouse_released)
        .mouse_moved(mouse_moved)
        .build()
        .unwrap();
    let window = app.window(_window).unwrap();
    let egui = Egui::from_window(&window);
    let mut world = World::new(Vec2::new(0.0, -18.0), ITERATIONS);
    // Give the ball a solid bounce off walls/flippers so an in-plane hit keeps
    // its speed instead of dead-stopping (this is what made the ball feel slow
    // after touching a flipper).
    world.set_restitution(0.35);

    let editor = EditorState {
        tool: EditTool::Select,
        selected_id: None,
        drag_start: None,
        dragging: false,
        rotating: false,
        wall_width: 2.0,
        wall_height: 0.5,
        bumper_radius: 0.7,
        bumper_boost: 16.0,
        bumper_score: 100,
        chain_links: 8,
        chain_length: 4.0,
        chain_end_mass: 15.0,
        fluid_radius: 2.0,
        fluid_type: FluidType::Water,
        fluid_viscosity: 0.8,
        bridge_segments: 10,
        bridge_softness: 0.02,
        target_width: 1.2,
        target_height: 0.35,
        target_score: 250,
        drain_width: 5.0,
        next_color_idx: 0,
    };

    let play = PlayState {
        score: 0,
        balls_remaining: 3,
        ball_body_id: None,
        flipper_left_id: None,
        flipper_right_id: None,
        flipper_left_rest: 0.0,
        flipper_left_up: 0.0,
        flipper_right_rest: 0.0,
        flipper_right_up: 0.0,
        plunger_body_id: None,
        plunger_charge: 0.0,
        plunger_charging: false,
        flipper_left_active: false,
        flipper_right_active: false,
        chain_bodies: Vec::new(),
        chain_anchor_id: None,
        metaball_bodies: Vec::new(),
        fluid_drag_active: false,
        game_over: false,
        high_score: 0,
        bumper_flash_timers: Vec::new(),
    };

    Model {
        _window,
        time_step: 1.0 / 60.0,
        world,
        egui,
        settings: EguiSettings {
            scale: 24.0,
            cam_x: 0.0,
            cam_y: 8.5,
            show_grid: true,
            snap_to_grid: true,
            grid_size: GRID_SNAP,
            show_contacts: false,
            metaball_resolution: 40,
            metaball_threshold: 0.5,
        },
        mode: GameMode::Editor,
        elements: Vec::new(),
        editor,
        play,
        next_id: 1,
        board_name_input: String::from("my_board"),
        load_board_flag: false,
        save_board_flag: false,
        mouse_world: Vec2::new(0.0, 0.0),
        is_first_frame: true,
    }
}

fn screen_to_world(app: &App, settings: &EguiSettings) -> Vec2 {
    let mouse_x = app.mouse.x;
    let mouse_y = app.mouse.y;
    Vec2::new(
        mouse_x / settings.scale + settings.cam_x,
        mouse_y / settings.scale + settings.cam_y,
    )
}

fn snap_to_grid(settings: &EguiSettings, p: Vec2) -> Vec2 {
    if settings.snap_to_grid {
        Vec2::new(
            snap_value(p.x, settings.grid_size),
            snap_value(p.y, settings.grid_size),
        )
    } else {
        p
    }
}

fn spawn_ball(model: &mut Model) {
    let mut ball = Body::new_circle(BALL_RADIUS, 1.5);
    ball.friction = 0.02;

    let spawn_pos = model
        .elements
        .iter()
        .find(|e| matches!(e.kind, ElementKind::BallSpawn))
        .map(|e| e.position)
        .unwrap_or_else(default_ball_spawn);

    ball.position = spawn_pos;
    model.play.ball_body_id = Some(ball.id);
    model.world.add_body(ball);
}

fn add_plunger(model: &mut Model) {
    // Plunger resting at the bottom of plunger lane (x: 6.6..7.8, y: -3.2)
    let mut plunger = Body::new(Vec2::new(1.1, 0.8), f32::MAX);
    plunger.position = Vec2::new(7.2, -3.2);
    plunger.friction = 0.05;
    model.play.plunger_body_id = Some(plunger.id);
    model.world.add_body(plunger);
}

fn add_cabinet_walls(model: &mut Model) {
    // Bottom Drain Floor
    let mut floor = Body::new(Vec2::new(20.0, 1.0), f32::MAX);
    floor.position = Vec2::new(0.0, -5.0);
    floor.friction = 0.2;
    model.world.add_body(floor);

    // Left Main Outer Wall
    let mut left_wall = Body::new(Vec2::new(0.6, 26.0), f32::MAX);
    left_wall.position = Vec2::new(-8.3, 9.0);
    left_wall.friction = 0.05;
    model.world.add_body(left_wall);

    // Right Outer Wall (outside plunger lane)
    let mut right_outer_wall = Body::new(Vec2::new(0.6, 26.0), f32::MAX);
    right_outer_wall.position = Vec2::new(8.3, 9.0);
    right_outer_wall.friction = 0.05;
    model.world.add_body(right_outer_wall);

    // Plunger Lane Divider Wall (separates main field from plunger lane x: 6.1..6.6)
    let mut plunger_divider = Body::new(Vec2::new(0.5, 17.0), f32::MAX);
    plunger_divider.position = Vec2::new(6.1, 4.5);
    plunger_divider.friction = 0.05;
    model.world.add_body(plunger_divider);

    // Top Main Curved Arch using smooth convex polygon segments
    let arch_center = Vec2::new(-0.5, 17.0);
    let num_segments = 12;
    let radius_outer = 8.8;
    let radius_inner = 8.2;

    for i in 0..num_segments {
        let theta1 = std::f32::consts::PI * (i as f32 / num_segments as f32);
        let theta2 = std::f32::consts::PI * ((i + 1) as f32 / num_segments as f32);

        let p1_inner =
            arch_center + Vec2::new(radius_inner * theta1.cos(), radius_inner * theta1.sin());
        let p2_inner =
            arch_center + Vec2::new(radius_inner * theta2.cos(), radius_inner * theta2.sin());
        let p2_outer =
            arch_center + Vec2::new(radius_outer * theta2.cos(), radius_outer * theta2.sin());
        let p1_outer =
            arch_center + Vec2::new(radius_outer * theta1.cos(), radius_outer * theta1.sin());

        let poly_verts = vec![p1_inner, p2_inner, p2_outer, p1_outer];
        let mut arch_seg = Body::new_polygon(poly_verts, f32::MAX);
        arch_seg.friction = 0.05;
        model.world.add_body(arch_seg);
    }

    // Slanted Lower Inlane Guides (guiding ball to flippers)
    // Left Inlane Guide
    let mut left_inlane = Body::new(Vec2::new(6.5, 0.5), f32::MAX);
    left_inlane.position = Vec2::new(-6.2, 2.2);
    left_inlane.rotation = -0.55;
    left_inlane.friction = 0.1;
    model.world.add_body(left_inlane);

    // Right Inlane Guide (leading to right flipper)
    let mut right_inlane = Body::new(Vec2::new(5.0, 0.5), f32::MAX);
    right_inlane.position = Vec2::new(3.6, 1.8);
    right_inlane.rotation = 0.55;
    right_inlane.friction = 0.1;
    model.world.add_body(right_inlane);
}

fn populate_default_board(model: &mut Model) {
    model.elements.clear();

    // 3 Bumpers in triangle formation at upper playfield
    model.elements.push(BoardElement {
        id: model.next_id,
        position: Vec2::new(-2.0, 14.0),
        rotation: 0.0,
        kind: ElementKind::Bumper {
            radius: 0.8,
            boost: 22.0,
            score: 100,
        },
        color: [0.95, 0.35, 0.2],
    });
    model.next_id += 1;

    model.elements.push(BoardElement {
        id: model.next_id,
        position: Vec2::new(2.0, 14.0),
        rotation: 0.0,
        kind: ElementKind::Bumper {
            radius: 0.8,
            boost: 22.0,
            score: 100,
        },
        color: [0.95, 0.35, 0.2],
    });
    model.next_id += 1;

    model.elements.push(BoardElement {
        id: model.next_id,
        position: Vec2::new(0.0, 17.0),
        rotation: 0.0,
        kind: ElementKind::Bumper {
            radius: 0.9,
            boost: 26.0,
            score: 250,
        },
        color: [1.0, 0.8, 0.1],
    });
    model.next_id += 1;

    // Drop Targets on sides
    model.elements.push(BoardElement {
        id: model.next_id,
        position: Vec2::new(-6.0, 11.0),
        rotation: 0.3,
        kind: ElementKind::Target {
            width: 1.2,
            height: 0.35,
            score: 500,
        },
        color: [0.2, 0.8, 0.9],
    });
    model.next_id += 1;

    model.elements.push(BoardElement {
        id: model.next_id,
        position: Vec2::new(4.2, 11.0),
        rotation: -0.3,
        kind: ElementKind::Target {
            width: 1.2,
            height: 0.35,
            score: 500,
        },
        color: [0.2, 0.8, 0.9],
    });
    model.next_id += 1;

    // Ball Spawn marker in plunger lane
    model.elements.push(BoardElement {
        id: model.next_id,
        position: default_ball_spawn(),
        rotation: 0.0,
        kind: ElementKind::BallSpawn,
        color: [0.9, 0.9, 0.9],
    });
    model.next_id += 1;

    // Flippers (pivots at the standard flipper spots)
    model.elements.push(BoardElement {
        id: model.next_id,
        position: Vec2::new(-3.2, -0.6),
        rotation: -0.4,
        kind: ElementKind::Flipper {
            side: FlipperSide::Left,
            length: FLIPPER_LENGTH,
        },
        color: [0.1, 0.75, 0.5],
    });
    model.next_id += 1;

    model.elements.push(BoardElement {
        id: model.next_id,
        position: Vec2::new(1.6, -0.6),
        rotation: 0.4,
        kind: ElementKind::Flipper {
            side: FlipperSide::Right,
            length: FLIPPER_LENGTH,
        },
        color: [0.1, 0.75, 0.5],
    });
    model.next_id += 1;
}

fn build_element_bodies(model: &mut Model) {
    let elements = model.elements.clone();
    for elem in &elements {
        match &elem.kind {
            ElementKind::Wall { width, height } => {
                let mut body = Body::new(Vec2::new(*width, *height), f32::MAX);
                body.position = elem.position;
                body.rotation = elem.rotation;
                body.friction = 0.3;
                model.world.add_body(body);
            }
            ElementKind::Bumper {
                radius,
                boost: _,
                score: _,
            } => {
                let mut body = Body::new_circle(*radius, f32::MAX);
                body.position = elem.position;
                body.friction = 0.1;
                model.world.add_body(body);
            }
            ElementKind::Flipper { side, length } => {
                let pivot = elem.position;
                let offset_x = match side {
                    FlipperSide::Left => length * 0.45,
                    FlipperSide::Right => -length * 0.45,
                };
                let rot_init = elem.rotation;
                let rot_mat = sylt_2d::math_utils::Mat2x2::new_from_angle(rot_init);
                let mut flipper = Body::new(Vec2::new(*length, FLIPPER_WIDTH), 15.0);
                flipper.position = pivot + rot_mat * Vec2::new(offset_x, 0.0);
                flipper.friction = 0.6;
                flipper.rotation = rot_init;
                let body_id = flipper.id;
                match side {
                    FlipperSide::Left => {
                        model.play.flipper_left_id = Some(body_id);
                        model.play.flipper_left_rest = rot_init;
                        model.play.flipper_left_up = rot_init + FLIPPER_UP_DELTA;
                    }
                    FlipperSide::Right => {
                        model.play.flipper_right_id = Some(body_id);
                        model.play.flipper_right_rest = rot_init;
                        model.play.flipper_right_up = rot_init - FLIPPER_UP_DELTA;
                    }
                }
                model.world.add_body(flipper.clone());

                let mut anchor = Body::new(Vec2::new(0.1, 0.1), f32::MAX);
                anchor.position = pivot;
                model.world.add_body(anchor.clone());

                let mut joint = Joint::new(anchor, flipper, pivot, &model.world);
                // Stiff hinge: keeps the flipper from bobbing around its pivot,
                // which previously showed up as visible jitter in place.
                joint.softness = 0.005;
                model.world.add_joint(joint);
            }
            ElementKind::Chain {
                link_count,
                total_length,
                end_mass,
            } => {
                let link_height = total_length / *link_count as f32;
                let link_w = 0.3;
                let mut prev_body: Option<Body> = None;

                for i in 0..*link_count {
                    let x = elem.position.x;
                    let y = elem.position.y - i as f32 * link_height;
                    let is_last = i == *link_count - 1;
                    let mass = if is_last { *end_mass } else { 2.0 };
                    let mut link = Body::new(Vec2::new(link_w, link_height * 0.9), mass);
                    link.position = Vec2::new(x, y);
                    link.friction = 0.3;
                    model.play.chain_bodies.push(link.id);
                    model.world.add_body(link.clone());

                    let link_clone = link.clone();
                    if i == 0 {
                        let mut anchor = Body::new(Vec2::new(0.2, 0.2), f32::MAX);
                        anchor.position = elem.position;
                        model.play.chain_anchor_id = Some(anchor.id);
                        model.world.add_body(anchor.clone());
                        let joint = Joint::new(anchor, link, elem.position, &model.world);
                        model.world.add_joint(joint);
                    } else if let Some(ref prev) = prev_body {
                        let anchor_pt = Vec2::new(x, y + link_height * 0.5);
                        let joint = Joint::new(prev.clone(), link, anchor_pt, &model.world);
                        model.world.add_joint(joint);
                    }
                    prev_body = Some(link_clone);
                }
            }
            ElementKind::FluidPool {
                radius,
                fluid: _,
                viscosity: _,
            } => {
                let mut metaball = Body::new_circle(*radius, f32::MAX);
                metaball.position = elem.position;
                metaball.friction = 0.1;
                model.play.metaball_bodies.push(metaball.id);
                model.world.add_body(metaball);
            }
            ElementKind::SoftBridge {
                end_x,
                segments,
                softness,
            } => {
                let start = elem.position;
                let end = Vec2::new(*end_x, elem.position.y);
                let total_dist = (end - start).length();
                let seg_len = total_dist / *segments as f32;
                let seg_h = 0.2;

                let mass = 1.0;
                let frequency_hz = 2.0;
                let damping_ratio = 0.5;
                let omega = 2.0 * std::f32::consts::PI * frequency_hz;
                let d = 2.0 * mass * damping_ratio * omega;
                let k = mass * omega * omega;
                let time_step = model.time_step;
                let bias_factor = time_step * k / (d + time_step * k);

                let dir = if total_dist > f32::EPSILON {
                    Vec2::new(
                        (end.x - start.x) / total_dist,
                        (end.y - start.y) / total_dist,
                    )
                } else {
                    Vec2::new(1.0, 0.0)
                };

                let mut prev_plank: Option<Body> = None;

                for i in 0..*segments {
                    let t = (i as f32 + 0.5) / *segments as f32;
                    let pos = Vec2::new(
                        start.x + dir.x * t * total_dist,
                        start.y + dir.y * t * total_dist,
                    );
                    let mut plank = Body::new(Vec2::new(seg_len * 0.95, seg_h), mass);
                    plank.position = pos;
                    plank.friction = 0.5;
                    model.world.add_body(plank.clone());

                    if i == 0 {
                        let mut anchor = Body::new(Vec2::new(0.1, 0.1), f32::MAX);
                        anchor.position = start;
                        model.world.add_body(anchor.clone());
                        let mut joint = Joint::new(anchor, plank.clone(), start, &model.world);
                        joint.softness = *softness;
                        joint.bias_factor = bias_factor;
                        model.world.add_joint(joint);
                    } else if let Some(ref prev) = prev_plank {
                        let joint_pt = Vec2::new(
                            start.x + dir.x * (i as f32 / *segments as f32) * total_dist,
                            start.y + dir.y * (i as f32 / *segments as f32) * total_dist,
                        );
                        let mut joint =
                            Joint::new(prev.clone(), plank.clone(), joint_pt, &model.world);
                        joint.softness = *softness;
                        joint.bias_factor = bias_factor;
                        model.world.add_joint(joint);
                    }
                    prev_plank = Some(plank);
                }

                if let Some(ref last_plank) = prev_plank {
                    let mut end_anchor = Body::new(Vec2::new(0.1, 0.1), f32::MAX);
                    end_anchor.position = end;
                    model.world.add_body(end_anchor.clone());
                    let mut joint = Joint::new(end_anchor, last_plank.clone(), end, &model.world);
                    joint.softness = *softness;
                    joint.bias_factor = bias_factor;
                    model.world.add_joint(joint);
                }
            }
            ElementKind::Target {
                width,
                height,
                score: _,
            } => {
                let mut body = Body::new(Vec2::new(*width, *height), f32::MAX);
                body.position = elem.position;
                body.rotation = elem.rotation;
                body.friction = 0.5;
                model.world.add_body(body);
            }
            ElementKind::Drain { width } => {
                let mut body = Body::new(Vec2::new(*width, 0.3), f32::MAX);
                body.position = elem.position;
                body.friction = 0.0;
                model.world.add_body(body);
            }
            ElementKind::BallSpawn => {}
        }
    }
}

fn enter_play_mode(model: &mut Model) {
    model.world.clear();
    model.play.score = 0;
    model.play.balls_remaining = 3;
    model.play.game_over = false;
    model.play.ball_body_id = None;
    model.play.flipper_left_id = None;
    model.play.flipper_right_id = None;
    model.play.flipper_left_rest = 0.0;
    model.play.flipper_left_up = 0.0;
    model.play.flipper_right_rest = 0.0;
    model.play.flipper_right_up = 0.0;
    model.play.plunger_body_id = None;
    model.play.plunger_charge = 0.0;
    model.play.flipper_left_active = false;
    model.play.flipper_right_active = false;
    model.play.chain_bodies.clear();
    model.play.chain_anchor_id = None;
    model.play.metaball_bodies.clear();
    model.play.fluid_drag_active = false;
    model.play.bumper_flash_timers.clear();

    if model.elements.is_empty() {
        populate_default_board(model);
    }

    add_cabinet_walls(model);
    add_plunger(model);
    build_element_bodies(model);
    spawn_ball(model);
}

fn enter_editor_mode(model: &mut Model) {
    model.world.clear();
    model.play.ball_body_id = None;
    model.play.flipper_left_id = None;
    model.play.flipper_right_id = None;
    model.play.plunger_body_id = None;
    model.play.chain_bodies.clear();
    model.play.chain_anchor_id = None;
    model.play.metaball_bodies.clear();
    if model.elements.is_empty() {
        populate_default_board(model);
    }
    // Keep the board frame (walls, arch, plunger) visible as a reference
    // while editing, so elements can be placed in context.
    add_cabinet_walls(model);
    add_plunger(model);
}

fn update(app: &App, model: &mut Model, _update: Update) {
    model.mouse_world = screen_to_world(app, &model.settings);

    if model.is_first_frame {
        model.is_first_frame = false;
        enter_play_mode(model);
        model.mode = GameMode::Play;
    }

    if model.mode == GameMode::Play && !model.play.game_over {
        apply_flippers(model);
        apply_plunger(model);
        let _ = model.world.step(model.time_step);
        apply_fluid_drag(model);
        detect_scoring(model);

        // Update visual flash effects
        model.play.bumper_flash_timers.retain_mut(|(_, timer)| {
            *timer -= model.time_step;
            *timer > 0.0
        });
    }

    model.egui.set_elapsed_time(_update.since_start);

    match model.mode {
        GameMode::Editor => {
            let mut switch_to_play = false;
            let mut delete_selected: Option<usize> = None;
            let mut do_save = false;
            let mut do_load = false;
            let mut do_clear = false;

            {
                let editor = &mut model.editor;
                let settings = &mut model.settings;
                let board_name = &mut model.board_name_input;
                let ctx = model.egui.begin_frame();

                egui::SidePanel::left("tools")
                    .default_width(220.0)
                    .show(&ctx, |ui| {
                        ui.heading("⚡ Pinball Board Editor");
                        ui.separator();
                        ui.label("Tools:");
                        ui.horizontal_wrapped(|ui| {
                            let tools = [
                                EditTool::Select,
                                EditTool::Wall,
                                EditTool::Bumper,
                                EditTool::FlipperLeft,
                                EditTool::FlipperRight,
                                EditTool::Chain,
                                EditTool::FluidPool,
                                EditTool::SoftBridge,
                                EditTool::Target,
                                EditTool::Drain,
                                EditTool::BallSpawn,
                                EditTool::Delete,
                            ];
                            for t in &tools {
                                let selected = editor.tool == *t;
                                if ui.selectable_label(selected, t.name()).clicked() {
                                    editor.tool = *t;
                                }
                            }
                        });
                        ui.separator();
                        ui.label("Element Properties:");
                        match editor.tool {
                            EditTool::Wall => {
                                ui.add(
                                    egui::Slider::new(&mut editor.wall_width, 0.5..=10.0)
                                        .text("Width"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut editor.wall_height, 0.2..=5.0)
                                        .text("Height"),
                                );
                            }
                            EditTool::Bumper => {
                                ui.add(
                                    egui::Slider::new(&mut editor.bumper_radius, 0.3..=2.0)
                                        .text("Radius"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut editor.bumper_boost, 3.0..=35.0)
                                        .text("Boost"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut editor.bumper_score, 50..=1000)
                                        .text("Score"),
                                );
                            }
                            EditTool::Chain => {
                                ui.add(
                                    egui::Slider::new(&mut editor.chain_links, 2..=20)
                                        .text("Links"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut editor.chain_length, 1.0..=10.0)
                                        .text("Length"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut editor.chain_end_mass, 5.0..=50.0)
                                        .text("End Mass"),
                                );
                            }
                            EditTool::FluidPool => {
                                ui.add(
                                    egui::Slider::new(&mut editor.fluid_radius, 0.5..=5.0)
                                        .text("Radius"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut editor.fluid_viscosity, 0.1..=3.0)
                                        .text("Viscosity"),
                                );
                                ui.horizontal(|ui| {
                                    for ft in [
                                        FluidType::Water,
                                        FluidType::Slime,
                                        FluidType::Lava,
                                        FluidType::Acid,
                                    ] {
                                        if ui
                                            .selectable_label(editor.fluid_type == ft, ft.name())
                                            .clicked()
                                        {
                                            editor.fluid_type = ft;
                                        }
                                    }
                                });
                            }
                            EditTool::SoftBridge => {
                                ui.add(
                                    egui::Slider::new(&mut editor.bridge_segments, 3..=20)
                                        .text("Segments"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut editor.bridge_softness, 0.005..=0.1)
                                        .text("Softness"),
                                );
                            }
                            EditTool::Target => {
                                ui.add(
                                    egui::Slider::new(&mut editor.target_width, 0.5..=3.0)
                                        .text("Width"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut editor.target_height, 0.1..=1.0)
                                        .text("Height"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut editor.target_score, 50..=2000)
                                        .text("Score"),
                                );
                            }
                            EditTool::Drain => {
                                ui.add(
                                    egui::Slider::new(&mut editor.drain_width, 1.0..=10.0)
                                        .text("Width"),
                                );
                            }
                            _ => {
                                ui.label("Click on board to place element.");
                            }
                        }
                        if let Some(sel_id) = editor.selected_id {
                            ui.separator();
                            ui.label(format!("Selected: #{}", sel_id));
                            if let Some(elem) = model.elements.iter_mut().find(|e| e.id == sel_id) {
                                ui.add(
                                    egui::Slider::new(
                                        &mut elem.rotation,
                                        -std::f32::consts::PI..=std::f32::consts::PI,
                                    )
                                    .text("Rotation"),
                                );
                            }
                            ui.label("Tip: drag the yellow handle on the");
                            ui.label("board to rotate the element.");
                            if ui.button("Delete Selected").clicked() {
                                delete_selected = Some(sel_id);
                            }
                        }
                        ui.separator();
                        ui.label("Board Management:");
                        ui.horizontal(|ui| {
                            ui.label("Name:");
                            ui.text_edit_singleline(board_name);
                        });
                        if ui.button("Save Board").clicked() {
                            do_save = true;
                        }
                        if ui.button("Load Board").clicked() {
                            do_load = true;
                        }
                        if ui.button("Clear All").clicked() {
                            do_clear = true;
                        }
                        ui.separator();
                        ui.label("View:");
                        ui.checkbox(&mut settings.show_grid, "Show Grid");
                        ui.checkbox(&mut settings.snap_to_grid, "Snap to Grid");
                        if settings.snap_to_grid {
                            ui.add(
                                egui::Slider::new(&mut settings.grid_size, 0.1..=1.0)
                                    .text("Grid Size"),
                            );
                        }
                        ui.checkbox(&mut settings.show_contacts, "Show Contacts");
                        ui.add(egui::Slider::new(&mut settings.scale, 10.0..=50.0).text("Zoom"));
                        ui.horizontal(|ui| {
                            ui.label("X:");
                            ui.add(egui::Slider::new(&mut settings.cam_x, -20.0..=20.0));
                        });
                        ui.horizontal(|ui| {
                            ui.label("Y:");
                            ui.add(egui::Slider::new(&mut settings.cam_y, -5.0..=25.0));
                        });
                        ui.separator();
                        if ui.button("▶ Play Test  [TAB]").clicked() {
                            switch_to_play = true;
                        }
                    });
            }

            if let Some(sel_id) = delete_selected {
                model.elements.retain(|e| e.id != sel_id);
                model.editor.selected_id = None;
            }
            if do_save {
                model.save_board_flag = true;
            }
            if do_load {
                model.load_board_flag = true;
            }
            if do_clear {
                model.elements.clear();
                model.editor.selected_id = None;
            }
            if switch_to_play {
                model.mode = GameMode::Play;
                enter_play_mode(model);
            }
        }
        GameMode::Play => {
            let mut switch_to_edit = false;
            let mut do_restart = false;

            {
                let settings = &mut model.settings;
                let play = &model.play;
                let ctx = model.egui.begin_frame();

                egui::SidePanel::right("play_hud")
                    .default_width(220.0)
                    .show(&ctx, |ui| {
                        ui.heading("🎰 PINBALL ARCADE");
                        ui.separator();
                        ui.label(
                            egui::RichText::new(format!("SCORE: {}", play.score))
                                .size(18.0)
                                .color(egui::Color32::YELLOW),
                        );
                        ui.label(format!("HIGH SCORE: {}", play.high_score));
                        ui.label(format!("BALLS REMAINING: {}", play.balls_remaining));
                        ui.separator();
                        ui.label("Controls:");
                        ui.label("Left Flipper:  A / Left");
                        ui.label("Right Flipper: D / Right");
                        ui.label("Plunger:       Space (Hold & Release)");
                        ui.label("Reset:         R");
                        ui.separator();
                        if play.game_over {
                            ui.colored_label(egui::Color32::RED, "💥 GAME OVER 💥");
                            if ui.button("🔄 Restart").clicked() {
                                do_restart = true;
                            }
                        }
                        ui.separator();
                        ui.label("View:");
                        ui.add(egui::Slider::new(&mut settings.scale, 10.0..=50.0).text("Zoom"));
                        ui.checkbox(&mut settings.show_contacts, "Show Contacts");
                        ui.separator();
                        if ui.button("✏ Edit Mode  [TAB]").clicked() {
                            switch_to_edit = true;
                        }
                    });
            }

            if do_restart {
                enter_play_mode(model);
            }
            if switch_to_edit {
                model.mode = GameMode::Editor;
                enter_editor_mode(model);
            }
        }
    }
}

fn apply_fluid_drag(model: &mut Model) {
    if let Some(ball_id) = model.play.ball_body_id {
        let ball_body_opt = model.world.bodies.iter().find(|b| b.borrow().id == ball_id);
        if let Some(ball_body) = ball_body_opt {
            let ball_pos = ball_body.borrow().position;
            let mut total_drag = Vec2::new(0.0, 0.0);
            let mut in_fluid = false;

            let elements = model.elements.clone();
            for elem in &elements {
                if let ElementKind::FluidPool {
                    radius,
                    fluid,
                    viscosity,
                } = &elem.kind
                {
                    let dist = (ball_pos - elem.position).length();
                    if dist < *radius {
                        in_fluid = true;
                        let ball = ball_body.borrow();
                        let drag_coeff = match fluid {
                            FluidType::Water => *viscosity * 1.5,
                            FluidType::Slime => *viscosity * 4.0,
                            FluidType::Lava => *viscosity * 1.0,
                            FluidType::Acid => *viscosity * 2.5,
                        };
                        total_drag = total_drag + ball.velocity * (-drag_coeff);
                        drop(ball);
                    }
                }
            }

            if in_fluid {
                let mut ball = ball_body.borrow_mut();
                ball.velocity = ball.velocity + total_drag * model.time_step;
                model.play.fluid_drag_active = true;
            } else {
                model.play.fluid_drag_active = false;
            }
        }
    }
}

fn detect_scoring(model: &mut Model) {
    if let Some(ball_id) = model.play.ball_body_id {
        let mut ball_pos = Vec2::new(0.0, 0.0);
        let mut ball_found = false;
        for body in model.world.bodies.iter() {
            if body.borrow().id == ball_id {
                ball_pos = body.borrow().position;
                ball_found = true;
                break;
            }
        }
        if !ball_found {
            return;
        }

        let elements = model.elements.clone();
        for elem in &elements {
            match &elem.kind {
                ElementKind::Bumper {
                    radius,
                    boost,
                    score,
                } => {
                    let dist = (elem.position - ball_pos).length();
                    if dist < *radius + BALL_RADIUS + 0.15 {
                        let body_opt = model.world.bodies.iter().find(|b| b.borrow().id == ball_id);
                        if let Some(body_ref) = body_opt {
                            let mut body = body_ref.borrow_mut();
                            let dir = vec2_normalize(body.position - elem.position);
                            // Avoid applying repeated boost if already moving away fast
                            if body.velocity.dot(dir) < *boost {
                                body.velocity = dir * *boost;
                                model.play.score += score;
                                model.play.bumper_flash_timers.push((elem.position, 0.2));
                            }
                        }
                    }
                }
                ElementKind::Target {
                    width,
                    height,
                    score,
                } => {
                    let dist = (elem.position - ball_pos).length();
                    if dist < (*width + *height) * 0.5 + BALL_RADIUS {
                        model.play.score += score / 10; // smooth continuous collision score
                    }
                }
                _ => {}
            }
        }

        // Drain condition: ball drops below flippers (y < -3.0)
        if ball_pos.y < -3.5 {
            model.play.balls_remaining = model.play.balls_remaining.saturating_sub(1);
            if model.play.balls_remaining == 0 {
                model.play.game_over = true;
                if model.play.score > model.play.high_score {
                    model.play.high_score = model.play.score;
                }
            } else {
                model.world.bodies.retain(|b| b.borrow().id != ball_id);
                model.play.ball_body_id = None;
                spawn_ball(model);
            }
        }
    }
}

fn apply_flippers(model: &mut Model) {
    let target_left = if model.play.flipper_left_active {
        model.play.flipper_left_up
    } else {
        model.play.flipper_left_rest
    };
    let target_right = if model.play.flipper_right_active {
        model.play.flipper_right_up
    } else {
        model.play.flipper_right_rest
    };
    let left_offset = Vec2::new(FLIPPER_LENGTH * 0.45, 0.0);
    let right_offset = Vec2::new(-FLIPPER_LENGTH * 0.45, 0.0);

    if let Some(left_id) = model.play.flipper_left_id {
        if let Some(body_ref) = model.world.bodies.iter().find(|b| b.borrow().id == left_id) {
            drive_flipper(&mut body_ref.borrow_mut(), target_left, left_offset);
        }
    }
    if let Some(right_id) = model.play.flipper_right_id {
        if let Some(body_ref) = model
            .world
            .bodies
            .iter()
            .find(|b| b.borrow().id == right_id)
        {
            drive_flipper(&mut body_ref.borrow_mut(), target_right, right_offset);
        }
    }
}

/// Drives a flipper toward a target rotation as a rigid body rotating about its
/// *pivot* (not its center).
///
/// A first-order servo sets the angular velocity proportional to the remaining
/// angle error (monotonic, never overshoots, never oscillates) and ALSO sets the
/// matching orbital linear velocity of the flipper center. Driving the angular
/// velocity alone makes the off-center hinge joint violently counteract it every
/// frame (felt as heaviness/sluggishness); keeping the pivot point stationary
/// gives the joint nothing to fight, so the flipper snaps crisply to its stop.
/// A tight dead-zone pins it exactly at rest and full-up.
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

    // Settle dead-zone: near the target and nearly stopped, snap to rest.
    if err.abs() < SETTLE_ANGLE && av.abs() < SETTLE_SPEED {
        body.angular_velocity = 0.0;
        body.velocity = Vec2::new(0.0, 0.0);
        body.rotation = target;
    }
}

fn apply_plunger(model: &mut Model) {
    if model.play.plunger_charging {
        model.play.plunger_charge = (model.play.plunger_charge + 1.2).min(PLUNGER_MAX_CHARGE);
    }
}

fn launch_plunger(model: &mut Model) {
    if model.play.plunger_charge > 0.0 {
        if let Some(ball_id) = model.play.ball_body_id {
            if let Some(body_ref) = model.world.bodies.iter().find(|b| b.borrow().id == ball_id) {
                let mut body = body_ref.borrow_mut();
                // If ball is in plunger lane (x > 5.5), launch it upwards with spring force
                if body.position.x > 5.5 && body.position.y < 3.0 {
                    body.velocity = Vec2::new(0.0, model.play.plunger_charge);
                }
            }
        }
        model.play.plunger_charge = 0.0;
        model.play.plunger_charging = false;
    }
}

fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    model.egui.handle_raw_event(event);
}

fn mouse_pressed(_app: &App, model: &mut Model, button: MouseButton) {
    if model.egui.ctx().is_pointer_over_area() {
        return;
    }

    if model.mode == GameMode::Editor && button == MouseButton::Left {
        let raw_pos = model.mouse_world;
        let pos = snap_to_grid(&model.settings, raw_pos);

        match model.editor.tool {
            EditTool::Select => {
                // Grab the rotation handle of the selected element before hit-testing.
                if let Some(sel_id) = model.editor.selected_id {
                    if let Some(elem) = model.elements.iter().find(|e| e.id == sel_id) {
                        if (raw_pos - rotation_handle_pos(elem)).length() < 0.35 {
                            model.editor.rotating = true;
                            model.editor.dragging = false;
                            model.editor.drag_start = None;
                            return;
                        }
                    }
                }
                let clicked_id = model.elements.iter().rev().find_map(|e| {
                    if element_hit(e, raw_pos) {
                        Some(e.id)
                    } else {
                        None
                    }
                });
                model.editor.selected_id = clicked_id;
                model.editor.drag_start = Some(raw_pos);
                model.editor.dragging = clicked_id.is_some();
            }
            EditTool::Delete => {
                let clicked_id = model.elements.iter().rev().find_map(|e| {
                    if element_hit(e, raw_pos) {
                        Some(e.id)
                    } else {
                        None
                    }
                });
                if let Some(id) = clicked_id {
                    model.elements.retain(|e| e.id != id);
                }
            }
            _ => {
                let color_idx = model.editor.next_color_idx;
                let colors = [
                    [0.95, 0.35, 0.2],
                    [0.2, 0.8, 0.9],
                    [0.3, 0.9, 0.4],
                    [0.95, 0.75, 0.1],
                    [0.85, 0.3, 0.85],
                    [0.3, 0.6, 0.95],
                ];
                let color = colors[color_idx % colors.len()];
                model.editor.next_color_idx += 1;

                let wall_width = model.editor.wall_width;
                let wall_height = model.editor.wall_height;
                let bumper_radius = model.editor.bumper_radius;
                let bumper_boost = model.editor.bumper_boost;
                let bumper_score = model.editor.bumper_score;
                let chain_links = model.editor.chain_links;
                let chain_length = model.editor.chain_length;
                let chain_end_mass = model.editor.chain_end_mass;
                let fluid_radius = model.editor.fluid_radius;
                let fluid_type = model.editor.fluid_type;
                let fluid_viscosity = model.editor.fluid_viscosity;
                let bridge_segments = model.editor.bridge_segments;
                let bridge_softness = model.editor.bridge_softness;
                let target_width = model.editor.target_width;
                let target_height = model.editor.target_height;
                let target_score = model.editor.target_score;
                let drain_width = model.editor.drain_width;

                let kind = match model.editor.tool {
                    EditTool::Wall => ElementKind::Wall {
                        width: wall_width,
                        height: wall_height,
                    },
                    EditTool::Bumper => ElementKind::Bumper {
                        radius: bumper_radius,
                        boost: bumper_boost,
                        score: bumper_score,
                    },
                    EditTool::FlipperLeft => ElementKind::Flipper {
                        side: FlipperSide::Left,
                        length: FLIPPER_LENGTH,
                    },
                    EditTool::FlipperRight => ElementKind::Flipper {
                        side: FlipperSide::Right,
                        length: FLIPPER_LENGTH,
                    },
                    EditTool::Chain => ElementKind::Chain {
                        link_count: chain_links,
                        total_length: chain_length,
                        end_mass: chain_end_mass,
                    },
                    EditTool::FluidPool => ElementKind::FluidPool {
                        radius: fluid_radius,
                        fluid: fluid_type,
                        viscosity: fluid_viscosity,
                    },
                    EditTool::SoftBridge => ElementKind::SoftBridge {
                        end_x: pos.x + 4.0,
                        segments: bridge_segments,
                        softness: bridge_softness,
                    },
                    EditTool::Target => ElementKind::Target {
                        width: target_width,
                        height: target_height,
                        score: target_score,
                    },
                    EditTool::Drain => ElementKind::Drain { width: drain_width },
                    EditTool::BallSpawn => ElementKind::BallSpawn,
                    _ => return,
                };

                let rotation = match model.editor.tool {
                    EditTool::FlipperLeft => -0.45,
                    EditTool::FlipperRight => 0.45,
                    _ => 0.0,
                };
                let id = model.next_id;
                model.next_id += 1;
                model.elements.push(BoardElement {
                    id,
                    position: pos,
                    rotation,
                    kind,
                    color,
                });
            }
        }
    }
}

fn mouse_released(_app: &App, model: &mut Model, _button: MouseButton) {
    if model.mode == GameMode::Editor {
        model.editor.dragging = false;
        model.editor.rotating = false;
        model.editor.drag_start = None;
    }
}

fn mouse_moved(_app: &App, model: &mut Model, _pos: Point2) {
    if model.mode != GameMode::Editor {
        return;
    }
    if model.editor.rotating {
        if let Some(sel_id) = model.editor.selected_id {
            if let Some(elem) = model.elements.iter_mut().find(|e| e.id == sel_id) {
                let delta = model.mouse_world - elem.position;
                elem.rotation = delta.y.atan2(delta.x);
            }
        }
        return;
    }
    if model.editor.dragging {
        if let Some(sel_id) = model.editor.selected_id {
            if let Some(elem) = model.elements.iter_mut().find(|e| e.id == sel_id) {
                let delta =
                    model.mouse_world - model.editor.drag_start.unwrap_or(model.mouse_world);
                let new_pos = elem.position + delta;
                elem.position = if model.settings.snap_to_grid {
                    Vec2::new(
                        snap_value(new_pos.x, model.settings.grid_size),
                        snap_value(new_pos.y, model.settings.grid_size),
                    )
                } else {
                    new_pos
                };
                model.editor.drag_start = Some(model.mouse_world);
            }
        }
    }
}

fn key_pressed(_app: &App, model: &mut Model, key: Key) {
    match key {
        Key::Tab => match model.mode {
            GameMode::Editor => {
                model.mode = GameMode::Play;
                enter_play_mode(model);
            }
            GameMode::Play => {
                model.mode = GameMode::Editor;
                enter_editor_mode(model);
            }
        },
        Key::A | Key::Left => {
            if model.mode == GameMode::Play {
                model.play.flipper_left_active = true;
            }
        }
        Key::D | Key::Right => {
            if model.mode == GameMode::Play {
                model.play.flipper_right_active = true;
            }
        }
        Key::Space => {
            if model.mode == GameMode::Play && !model.play.game_over {
                model.play.plunger_charging = true;
            }
        }
        Key::R => {
            if model.mode == GameMode::Play {
                enter_play_mode(model);
            }
        }
        Key::W => model.settings.cam_y += 1.5,
        Key::S => model.settings.cam_y -= 1.5,
        Key::Q => model.settings.cam_x -= 1.5,
        Key::E => model.settings.cam_x += 1.5,
        _ => {}
    }
}

fn key_released(_app: &App, model: &mut Model, key: Key) {
    match key {
        Key::A | Key::Left => {
            model.play.flipper_left_active = false;
        }
        Key::D | Key::Right => {
            model.play.flipper_right_active = false;
        }
        Key::Space => {
            launch_plunger(model);
        }
        _ => {}
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    let draw = draw
        .translate(vec3(
            -model.settings.cam_x * model.settings.scale,
            -model.settings.cam_y * model.settings.scale,
            0.0,
        ))
        .scale(model.settings.scale);

    // Deep Arcade Dark Background
    draw.background().color(rgb(0.04, 0.04, 0.07));

    // Playfield Felt Border Glow
    draw.rect()
        .x_y(-0.8, 8.5)
        .w_h(17.5, 27.5)
        .color(rgb(0.07, 0.08, 0.14))
        .stroke(rgb(0.2, 0.25, 0.45))
        .stroke_weight(0.15);

    if model.settings.show_grid {
        draw_grid(&draw, &model.settings);
    }

    if model.mode == GameMode::Editor {
        draw_editor_elements(model, &draw);
        draw_placement_preview(model, &draw);

        // Cursor crosshair + snapped coordinates readout.
        let snapped = snap_to_grid(&model.settings, model.mouse_world);
        draw.line()
            .start(pt2(snapped.x - 0.12, snapped.y))
            .end(pt2(snapped.x + 0.12, snapped.y))
            .weight(0.03)
            .color(rgba(1.0, 1.0, 0.4, 0.9));
        draw.line()
            .start(pt2(snapped.x, snapped.y - 0.12))
            .end(pt2(snapped.x, snapped.y + 0.12))
            .weight(0.03)
            .color(rgba(1.0, 1.0, 0.4, 0.9));
        draw_world_label(
            &draw,
            &model.settings,
            model.mouse_world + Vec2::new(0.6, 0.5),
            &format!("({:.2}, {:.2})", snapped.x, snapped.y),
            13.0,
            rgba(1.0, 1.0, 0.6, 0.9),
        );
    }

    for (num, body) in model.world.iter_bodies().enumerate() {
        let is_ball = Some(body.id) == model.play.ball_body_id;
        let is_plunger = Some(body.id) == model.play.plunger_body_id;
        let is_flipper_left = Some(body.id) == model.play.flipper_left_id;
        let is_flipper_right = Some(body.id) == model.play.flipper_right_id;
        let is_chain = model.play.chain_bodies.contains(&body.id);

        match body.shape {
            Shape::Box => {
                if is_plunger {
                    // Stylized Metallic Red Plunger
                    let charge_pct = model.play.plunger_charge / PLUNGER_MAX_CHARGE;
                    let p_color = rgb(0.9, 0.2 + charge_pct * 0.7, 0.1);
                    draw.rect()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.width.x, body.width.y)
                        .rotate(body.rotation)
                        .color(p_color)
                        .stroke(rgb(1.0, 0.8, 0.8))
                        .stroke_weight(0.04);
                } else if is_flipper_left || is_flipper_right {
                    // Vibrant Neon Flippers
                    let f_color = if (is_flipper_left && model.play.flipper_left_active)
                        || (is_flipper_right && model.play.flipper_right_active)
                    {
                        rgb(0.1, 0.95, 0.8) // High Neon Cyan when active
                    } else {
                        rgb(0.1, 0.75, 0.5) // Sleek Emerald Green resting
                    };

                    draw.rect()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.width.x, body.width.y)
                        .rotate(body.rotation)
                        .color(f_color)
                        .stroke(WHITE)
                        .stroke_weight(0.04);
                } else if is_chain {
                    draw.rect()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.width.x, body.width.y)
                        .rotate(body.rotation)
                        .color(rgb(0.65, 0.65, 0.75))
                        .stroke(rgb(0.9, 0.9, 1.0))
                        .stroke_weight(0.02);
                } else if num == 0 {
                    // Bottom Drain Floor
                    draw.rect()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.width.x, body.width.y)
                        .rotate(body.rotation)
                        .color(rgb(0.15, 0.05, 0.08));
                } else {
                    // Cabinet & Guide Walls
                    draw.rect()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.width.x, body.width.y)
                        .rotate(body.rotation)
                        .color(rgb(0.18, 0.22, 0.32))
                        .stroke(rgb(0.35, 0.45, 0.65))
                        .stroke_weight(0.03);
                }
            }
            Shape::ConvexPolygon => {
                let tuples: Vec<(f32, f32)> = body
                    .get_polygon()
                    .get_vertices()
                    .into_iter()
                    .map(Into::into)
                    .collect();
                draw.polygon()
                    .color(rgb(0.45, 0.35, 0.75))
                    .x_y(body.position.x, body.position.y)
                    .rotate(body.rotation)
                    .points(tuples);
            }
            Shape::Circle => {
                if is_ball {
                    // Chrome Metallic Pinball with Soft Glow Shadow
                    draw.ellipse()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.radius * 2.6, body.radius * 2.6)
                        .color(rgba(0.2, 0.5, 1.0, 0.25));

                    draw.ellipse()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.radius * 2.0, body.radius * 2.0)
                        .color(rgb(0.95, 0.97, 1.0));

                    // Highlight spec dot
                    draw.ellipse()
                        .x_y(
                            body.position.x + body.radius * 0.3,
                            body.position.y + body.radius * 0.3,
                        )
                        .w_h(body.radius * 0.6, body.radius * 0.6)
                        .color(WHITE);
                } else if model.play.metaball_bodies.contains(&body.id) {
                    draw.ellipse()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.radius * 2.0, body.radius * 2.0)
                        .color(rgba(0.2, 0.6, 0.9, 0.15));
                } else if body.inv_mass == 0.0 {
                    // Bumper Rendering
                    let position = body.position;
                    let is_flashing = model
                        .play
                        .bumper_flash_timers
                        .iter()
                        .any(|(pos, _)| (*pos - position).length() < 0.1);

                    let (b_col, ring_col) = if is_flashing {
                        (rgb(1.0, 1.0, 0.6), rgb(1.0, 0.9, 0.3)) // Flash bright yellow
                    } else {
                        (rgb(0.95, 0.35, 0.2), rgb(1.0, 0.6, 0.2))
                    };

                    draw.ellipse()
                        .x_y(position.x, position.y)
                        .w_h(body.radius * 2.3, body.radius * 2.3)
                        .color(rgba(b_col.red, b_col.green, b_col.blue, 0.2));

                    draw.ellipse()
                        .x_y(position.x, position.y)
                        .w_h(body.radius * 2.0, body.radius * 2.0)
                        .color(b_col)
                        .stroke(WHITE)
                        .stroke_weight(0.04);

                    draw.ellipse()
                        .x_y(position.x, position.y)
                        .w_h(body.radius * 1.3, body.radius * 1.3)
                        .color(ring_col);
                } else {
                    draw.ellipse()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.radius * 2.0, body.radius * 2.0)
                        .color(rgb(0.5, 0.5, 0.6));
                }
            }
        }
    }

    if !model.play.metaball_bodies.is_empty() {
        let metaballs: Vec<Metaball> = model
            .world
            .iter_bodies()
            .filter(|b| model.play.metaball_bodies.contains(&b.id))
            .map(|b| Metaball::new(Vec2::new(b.position.x, b.position.y), b.radius, 1.0))
            .collect();

        if !metaballs.is_empty() {
            let clusters = cluster_metaballs(&metaballs, model.settings.metaball_threshold);
            for cluster in &clusters {
                let (bounds_min, bounds_max) = compute_metaball_bounds(
                    &cluster.metaballs,
                    model.settings.metaball_threshold,
                    2.0,
                );

                let obstacle_ids: Vec<usize> = model.play.metaball_bodies.clone();
                let obstacles: Vec<Body> = model
                    .world
                    .iter_bodies()
                    .filter(|b| !obstacle_ids.contains(&b.id))
                    .map(|b| (*b).clone())
                    .collect();

                let debug = marching_squares_debug(
                    &cluster.metaballs,
                    bounds_min,
                    bounds_max,
                    model.settings.metaball_resolution,
                    model.settings.metaball_threshold,
                    &obstacles,
                );

                for poly in &debug.polygons {
                    if poly.len() >= 3 {
                        draw.polygon()
                            .x_y(0.0, 0.0)
                            .color(rgba(0.2, 0.6, 0.95, 0.55))
                            .points(poly.clone());
                    }
                }
                for chain_poly in &debug.open_chains {
                    if chain_poly.len() >= 3 {
                        draw.polygon()
                            .x_y(0.0, 0.0)
                            .color(rgba(0.2, 0.6, 0.95, 0.35))
                            .points(chain_poly.clone());
                    }
                }
            }
        }
    }

    if model.settings.show_contacts {
        for (_, arbiter) in model.world.arbiters.iter() {
            for contact in arbiter.contacts.iter() {
                if let Some(c) = contact {
                    draw.ellipse()
                        .x_y(c.position.x, c.position.y)
                        .radius(0.08)
                        .color(ORANGE);
                }
            }
        }
    }

    for joint in model.world.joints.iter() {
        let x1 = joint.body_1.borrow().position;
        let x2 = joint.body_2.borrow().position;
        draw.line()
            .start(pt2(x1.x, x1.y))
            .end(pt2(x2.x, x2.y))
            .weight(0.03)
            .color(rgba(0.5, 0.7, 1.0, 0.4));
    }

    draw.to_frame(app, &frame).unwrap();
    model.egui.draw_to_frame(&frame).unwrap();
}

fn draw_grid(draw: &nannou::Draw, settings: &EguiSettings) {
    let step = settings.grid_size.max(0.05);
    let x_range = -15.0f32..15.0f32;
    let y_range = -5.0f32..25.0f32;

    // Minor grid lines at the snap step.
    let mut gx = snap_value(x_range.start, step);
    while gx <= x_range.end {
        draw.line()
            .start(pt2(gx, y_range.start))
            .end(pt2(gx, y_range.end))
            .weight(0.008)
            .color(rgba(1.0, 1.0, 1.0, 0.05));
        gx += step;
    }
    let mut gy = snap_value(y_range.start, step);
    while gy <= y_range.end {
        draw.line()
            .start(pt2(x_range.start, gy))
            .end(pt2(x_range.end, gy))
            .weight(0.008)
            .color(rgba(1.0, 1.0, 1.0, 0.05));
        gy += step;
    }

    // Major grid lines every whole unit.
    for x in (x_range.start as i32)..=(x_range.end as i32) {
        draw.line()
            .start(pt2(x as f32, y_range.start))
            .end(pt2(x as f32, y_range.end))
            .weight(0.012)
            .color(rgba(1.0, 1.0, 1.0, 0.1));
    }
    for y in (y_range.start as i32)..=(y_range.end as i32) {
        draw.line()
            .start(pt2(x_range.start, y as f32))
            .end(pt2(x_range.end, y as f32))
            .weight(0.012)
            .color(rgba(1.0, 1.0, 1.0, 0.1));
    }

    // Axes
    draw.line()
        .start(pt2(0.0, y_range.start))
        .end(pt2(0.0, y_range.end))
        .weight(0.02)
        .color(rgba(0.4, 0.6, 1.0, 0.4));
    draw.line()
        .start(pt2(x_range.start, 0.0))
        .end(pt2(x_range.end, 0.0))
        .weight(0.02)
        .color(rgba(0.4, 0.6, 1.0, 0.4));
}

/// Draw a small text label anchored to a world-space position. The font size is
/// scaled to the current zoom so the label stays a reasonable on-screen size.
fn draw_world_label(
    draw: &nannou::Draw,
    settings: &EguiSettings,
    world_pos: Vec2,
    text: &str,
    px: f32,
    color: Rgba,
) {
    let font_size = ((px / settings.scale).round()).max(1.0) as u32;
    draw.text(text)
        .x_y(world_pos.x, world_pos.y)
        .font_size(font_size)
        .color(color)
        .align_text_middle_y()
        .center_justify();
}

fn draw_element_shape(elem: &BoardElement, draw: &nannou::Draw, alpha: f32) {
    let c = elem.color;
    let col = rgba(c[0], c[1], c[2], alpha);
    let stroke_col = rgba(1.0, 1.0, 1.0, alpha);

    match &elem.kind {
        ElementKind::Wall { width, height } => {
            draw.rect()
                .x_y(elem.position.x, elem.position.y)
                .w_h(*width, *height)
                .rotate(elem.rotation)
                .color(col)
                .stroke(stroke_col)
                .stroke_weight(0.03);
        }
        ElementKind::Bumper { radius, .. } => {
            draw.ellipse()
                .x_y(elem.position.x, elem.position.y)
                .w_h(*radius * 2.0, *radius * 2.0)
                .color(col)
                .stroke(stroke_col)
                .stroke_weight(0.03);
            draw.ellipse()
                .x_y(elem.position.x, elem.position.y)
                .w_h(*radius * 1.2, *radius * 1.2)
                .color(rgba(1.0, 1.0, 1.0, 0.3 * alpha));
        }
        ElementKind::Flipper { side, length } => {
            let offset_x = match side {
                FlipperSide::Left => *length * 0.45,
                FlipperSide::Right => -*length * 0.45,
            };
            let angle = elem.rotation;
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            let cx = elem.position.x + offset_x * cos_a;
            let cy = elem.position.y + offset_x * sin_a;
            draw.rect()
                .x_y(cx, cy)
                .w_h(*length, FLIPPER_WIDTH)
                .rotate(angle)
                .color(col)
                .stroke(stroke_col)
                .stroke_weight(0.03);
            // Pivot point
            draw.ellipse()
                .x_y(elem.position.x, elem.position.y)
                .radius(0.15)
                .color(rgba(1.0, 1.0, 1.0, alpha));
        }
        ElementKind::Chain {
            link_count,
            total_length,
            ..
        } => {
            let link_h = total_length / *link_count as f32;
            for i in 0..*link_count {
                let y = elem.position.y - i as f32 * link_h;
                draw.rect()
                    .x_y(elem.position.x, y)
                    .w_h(0.3, link_h * 0.85)
                    .color(col)
                    .stroke(stroke_col)
                    .stroke_weight(0.02);
            }
            draw.ellipse()
                .x_y(elem.position.x, elem.position.y)
                .radius(0.15)
                .color(rgba(1.0, 1.0, 1.0, alpha));
        }
        ElementKind::FluidPool { radius, fluid, .. } => {
            let fluid_color = match fluid {
                FluidType::Water => rgba(0.2, 0.5, 0.9, 0.4 * alpha),
                FluidType::Slime => rgba(0.3, 0.8, 0.2, 0.4 * alpha),
                FluidType::Lava => rgba(0.9, 0.3, 0.1, 0.4 * alpha),
                FluidType::Acid => rgba(0.7, 0.9, 0.1, 0.4 * alpha),
            };
            draw.ellipse()
                .x_y(elem.position.x, elem.position.y)
                .w_h(*radius * 2.0, *radius * 2.0)
                .color(fluid_color)
                .stroke(stroke_col)
                .stroke_weight(0.03);
            draw.ellipse()
                .x_y(elem.position.x, elem.position.y)
                .w_h(*radius * 0.6, *radius * 0.6)
                .color(rgba(1.0, 1.0, 1.0, 0.2 * alpha));
        }
        ElementKind::SoftBridge {
            end_x, segments, ..
        } => {
            let start = elem.position;
            let end = Vec2::new(*end_x, elem.position.y);
            draw.line()
                .start(pt2(start.x, start.y))
                .end(pt2(end.x, end.y))
                .weight(0.15)
                .color(col);
            draw.ellipse()
                .x_y(start.x, start.y)
                .radius(0.12)
                .color(rgba(1.0, 1.0, 1.0, alpha));
            draw.ellipse()
                .x_y(end.x, end.y)
                .radius(0.12)
                .color(rgba(1.0, 1.0, 1.0, alpha));
            for i in 0..*segments {
                let t = i as f32 / *segments as f32;
                let x = start.x + t * (end.x - start.x);
                draw.line()
                    .start(pt2(x, start.y - 0.15))
                    .end(pt2(x, start.y + 0.15))
                    .weight(0.04)
                    .color(rgba(1.0, 1.0, 1.0, 0.4 * alpha));
            }
        }
        ElementKind::Target { width, height, .. } => {
            draw.rect()
                .x_y(elem.position.x, elem.position.y)
                .w_h(*width, *height)
                .rotate(elem.rotation)
                .color(col)
                .stroke(stroke_col)
                .stroke_weight(0.03);
        }
        ElementKind::Drain { width } => {
            draw.rect()
                .x_y(elem.position.x, elem.position.y)
                .w_h(*width, 0.3)
                .color(rgba(0.8, 0.1, 0.1, 0.5 * alpha))
                .stroke(rgba(1.0, 0.0, 0.0, alpha))
                .stroke_weight(0.05);
        }
        ElementKind::BallSpawn => {
            draw.ellipse()
                .x_y(elem.position.x, elem.position.y)
                .radius(BALL_RADIUS)
                .color(rgba(1.0, 1.0, 1.0, 0.5 * alpha))
                .stroke(stroke_col)
                .stroke_weight(0.03);
        }
    }
}

fn draw_editor_elements(model: &Model, draw: &nannou::Draw) {
    for elem in &model.elements {
        draw_element_shape(elem, draw, 1.0);

        // Small labels for scoring/identifier elements.
        match &elem.kind {
            ElementKind::Target { score, .. } => {
                draw_world_label(
                    draw,
                    &model.settings,
                    elem.position + Vec2::new(0.0, -0.45),
                    &score.to_string(),
                    13.0,
                    rgba(1.0, 1.0, 1.0, 0.9),
                );
            }
            ElementKind::BallSpawn => {
                draw_world_label(
                    draw,
                    &model.settings,
                    elem.position + Vec2::new(0.0, -0.45),
                    "S",
                    13.0,
                    rgba(1.0, 1.0, 1.0, 0.9),
                );
            }
            _ => {}
        }

        if Some(elem.id) == model.editor.selected_id {
            let pulse = 0.7;
            match &elem.kind {
                ElementKind::Bumper { radius, .. } => {
                    draw.ellipse()
                        .x_y(elem.position.x, elem.position.y)
                        .w_h((*radius + 0.2) * 2.0, (*radius + 0.2) * 2.0)
                        .no_fill()
                        .stroke(rgba(1.0, 1.0, 0.0, pulse))
                        .stroke_weight(0.05);
                }
                _ => {
                    draw.rect()
                        .x_y(elem.position.x, elem.position.y)
                        .w_h(2.0, 1.5)
                        .no_fill()
                        .stroke(rgba(1.0, 1.0, 0.0, pulse))
                        .stroke_weight(0.05);
                }
            }

            // Show the flipper's flip range (rest -> up target) so the user can
            // see what rotation will cause during play.
            if let ElementKind::Flipper { side, length } = &elem.kind {
                let offset_x = match side {
                    FlipperSide::Left => length * 0.45,
                    FlipperSide::Right => -length * 0.45,
                };
                let up_angle = match side {
                    FlipperSide::Left => elem.rotation + FLIPPER_UP_DELTA,
                    FlipperSide::Right => elem.rotation - FLIPPER_UP_DELTA,
                };
                let rest_tip = elem.position + rotate_vec(Vec2::new(offset_x, 0.0), elem.rotation);
                let up_tip = elem.position + rotate_vec(Vec2::new(offset_x, 0.0), up_angle);
                draw.line()
                    .start(pt2(elem.position.x, elem.position.y))
                    .end(pt2(rest_tip.x, rest_tip.y))
                    .weight(0.02)
                    .color(rgba(0.6, 0.9, 0.6, 0.6));
                draw.line()
                    .start(pt2(elem.position.x, elem.position.y))
                    .end(pt2(up_tip.x, up_tip.y))
                    .weight(0.02)
                    .color(rgba(0.9, 0.6, 0.6, 0.6));
            }

            // Rotation gizmo: axis line + draggable handle + live angle readout.
            let handle = rotation_handle_pos(elem);
            draw.line()
                .start(pt2(elem.position.x, elem.position.y))
                .end(pt2(handle.x, handle.y))
                .weight(0.04)
                .color(rgba(1.0, 1.0, 0.0, 0.8));
            draw.ellipse()
                .x_y(handle.x, handle.y)
                .radius(0.17)
                .color(rgba(1.0, 1.0, 0.0, 0.9))
                .stroke(rgba(1.0, 1.0, 1.0, 0.9))
                .stroke_weight(0.03);
            draw.ellipse()
                .x_y(handle.x, handle.y)
                .radius(0.06)
                .color(rgba(0.0, 0.0, 0.0, 0.6));
            let deg = elem.rotation.to_degrees();
            draw_world_label(
                draw,
                &model.settings,
                handle + Vec2::new(0.0, 0.35),
                &format!("{:.0}°", deg),
                13.0,
                rgba(1.0, 1.0, 0.2, 0.95),
            );
        }
    }
}

fn draw_placement_preview(model: &Model, draw: &nannou::Draw) {
    if model.editor.tool == EditTool::Select || model.editor.tool == EditTool::Delete {
        return;
    }

    let pos = snap_to_grid(&model.settings, model.mouse_world);

    let kind = match model.editor.tool {
        EditTool::Wall => ElementKind::Wall {
            width: model.editor.wall_width,
            height: model.editor.wall_height,
        },
        EditTool::Bumper => ElementKind::Bumper {
            radius: model.editor.bumper_radius,
            boost: model.editor.bumper_boost,
            score: model.editor.bumper_score,
        },
        EditTool::FlipperLeft => ElementKind::Flipper {
            side: FlipperSide::Left,
            length: FLIPPER_LENGTH,
        },
        EditTool::FlipperRight => ElementKind::Flipper {
            side: FlipperSide::Right,
            length: FLIPPER_LENGTH,
        },
        EditTool::Chain => ElementKind::Chain {
            link_count: model.editor.chain_links,
            total_length: model.editor.chain_length,
            end_mass: model.editor.chain_end_mass,
        },
        EditTool::FluidPool => ElementKind::FluidPool {
            radius: model.editor.fluid_radius,
            fluid: model.editor.fluid_type,
            viscosity: model.editor.fluid_viscosity,
        },
        EditTool::SoftBridge => ElementKind::SoftBridge {
            end_x: pos.x + 4.0,
            segments: model.editor.bridge_segments,
            softness: model.editor.bridge_softness,
        },
        EditTool::Target => ElementKind::Target {
            width: model.editor.target_width,
            height: model.editor.target_height,
            score: model.editor.target_score,
        },
        EditTool::Drain => ElementKind::Drain {
            width: model.editor.drain_width,
        },
        EditTool::BallSpawn => ElementKind::BallSpawn,
        _ => return,
    };

    let rotation = match model.editor.tool {
        EditTool::FlipperLeft => -0.45,
        EditTool::FlipperRight => 0.45,
        _ => 0.0,
    };

    let ghost = BoardElement {
        id: usize::MAX,
        position: pos,
        rotation,
        kind,
        color: [0.5, 0.9, 1.0],
    };
    draw_element_shape(&ghost, draw, 0.35);

    let extent = element_extent(&ghost);
    draw.rect()
        .x_y(pos.x, pos.y)
        .w_h((extent + 0.5) * 2.0, (extent + 0.5) * 2.0)
        .no_fill()
        .stroke(rgba(0.6, 0.9, 1.0, 0.4))
        .stroke_weight(0.02);
    draw_world_label(
        draw,
        &model.settings,
        pos + Vec2::new(0.0, extent + 0.7),
        &format!("({:.2}, {:.2})", pos.x, pos.y),
        13.0,
        rgba(0.7, 0.95, 1.0, 0.95),
    );
}
