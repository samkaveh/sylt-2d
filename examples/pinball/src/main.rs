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
const PLUNGER_MAX_CHARGE: f32 = 45.0;

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
    // Give the ball a little bounce off walls/flippers so it doesn't dead-stop.
    world.set_restitution(0.15);

    let editor = EditorState {
        tool: EditTool::Select,
        selected_id: None,
        drag_start: None,
        dragging: false,
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
            show_grid: false,
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

        let p1_inner = arch_center + Vec2::new(radius_inner * theta1.cos(), radius_inner * theta1.sin());
        let p2_inner = arch_center + Vec2::new(radius_inner * theta2.cos(), radius_inner * theta2.sin());
        let p2_outer = arch_center + Vec2::new(radius_outer * theta2.cos(), radius_outer * theta2.sin());
        let p1_outer = arch_center + Vec2::new(radius_outer * theta1.cos(), radius_outer * theta1.sin());

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

fn spawn_flippers(model: &mut Model) {
    // Left Flipper Pivot: (-3.2, -0.6). Flipper extends right towards center.
    let pivot_left = Vec2::new(-3.2, -0.6);
    let mut flipper_l = Body::new(Vec2::new(FLIPPER_LENGTH, FLIPPER_WIDTH), 15.0);
    // At rotation = 0, box is centered at position. We want pivot to be at left edge of box (-FLIPPER_LENGTH/2, 0).
    // Position of body center = pivot_left + Vec2(FLIPPER_LENGTH/2, 0) rotated by initial angle.
    let init_rot_l = -0.4;
    let local_offset_l = Vec2::new(FLIPPER_LENGTH * 0.45, 0.0);
    let rot_mat_l = sylt_2d::math_utils::Mat2x2::new_from_angle(init_rot_l);
    flipper_l.position = pivot_left + rot_mat_l * local_offset_l;
    flipper_l.friction = 0.8;
    flipper_l.rotation = init_rot_l;
    model.play.flipper_left_id = Some(flipper_l.id);
    model.world.add_body(flipper_l.clone());

    let mut anchor_l = Body::new(Vec2::new(0.1, 0.1), f32::MAX);
    anchor_l.position = pivot_left;
    model.world.add_body(anchor_l.clone());

    let mut joint_l = Joint::new(anchor_l, flipper_l, pivot_left, &model.world);
    joint_l.softness = 0.02;
    model.world.add_joint(joint_l);

    // Right Flipper Pivot: (1.6, -0.6). Flipper extends left towards center.
    let pivot_right = Vec2::new(1.6, -0.6);
    let mut flipper_r = Body::new(Vec2::new(FLIPPER_LENGTH, FLIPPER_WIDTH), 15.0);
    let init_rot_r = 0.4;
    let local_offset_r = Vec2::new(-FLIPPER_LENGTH * 0.45, 0.0);
    let rot_mat_r = sylt_2d::math_utils::Mat2x2::new_from_angle(init_rot_r);
    flipper_r.position = pivot_right + rot_mat_r * local_offset_r;
    flipper_r.friction = 0.8;
    flipper_r.rotation = init_rot_r;
    model.play.flipper_right_id = Some(flipper_r.id);
    model.world.add_body(flipper_r.clone());

    let mut anchor_r = Body::new(Vec2::new(0.1, 0.1), f32::MAX);
    anchor_r.position = pivot_right;
    model.world.add_body(anchor_r.clone());

    let mut joint_r = Joint::new(anchor_r, flipper_r, pivot_right, &model.world);
    joint_r.softness = 0.02;
    model.world.add_joint(joint_r);
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
                let rot_init = match side {
                    FlipperSide::Left => -0.45,
                    FlipperSide::Right => 0.45,
                };
                let mut flipper = Body::new(Vec2::new(*length, FLIPPER_WIDTH), 15.0);
                flipper.position = pivot + Vec2::new(offset_x, 0.0);
                flipper.friction = 0.6;
                flipper.rotation = rot_init;
                let body_id = flipper.id;
                match side {
                    FlipperSide::Left => model.play.flipper_left_id = Some(body_id),
                    FlipperSide::Right => model.play.flipper_right_id = Some(body_id),
                }
                model.world.add_body(flipper.clone());

                let mut anchor = Body::new(Vec2::new(0.1, 0.1), f32::MAX);
                anchor.position = pivot;
                model.world.add_body(anchor.clone());

                let mut joint = Joint::new(anchor, flipper, pivot, &model.world);
                joint.softness = 0.02;
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
                    Vec2::new((end.x - start.x) / total_dist, (end.y - start.y) / total_dist)
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
                        let mut joint = Joint::new(prev.clone(), plank.clone(), joint_pt, &model.world);
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
    spawn_flippers(model);
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
                                ui.add(egui::Slider::new(&mut editor.wall_width, 0.5..=10.0).text("Width"));
                                ui.add(egui::Slider::new(&mut editor.wall_height, 0.2..=5.0).text("Height"));
                            }
                            EditTool::Bumper => {
                                ui.add(egui::Slider::new(&mut editor.bumper_radius, 0.3..=2.0).text("Radius"));
                                ui.add(egui::Slider::new(&mut editor.bumper_boost, 3.0..=35.0).text("Boost"));
                                ui.add(egui::Slider::new(&mut editor.bumper_score, 50..=1000).text("Score"));
                            }
                            EditTool::Chain => {
                                ui.add(egui::Slider::new(&mut editor.chain_links, 2..=20).text("Links"));
                                ui.add(egui::Slider::new(&mut editor.chain_length, 1.0..=10.0).text("Length"));
                                ui.add(egui::Slider::new(&mut editor.chain_end_mass, 5.0..=50.0).text("End Mass"));
                            }
                            EditTool::FluidPool => {
                                ui.add(egui::Slider::new(&mut editor.fluid_radius, 0.5..=5.0).text("Radius"));
                                ui.add(egui::Slider::new(&mut editor.fluid_viscosity, 0.1..=3.0).text("Viscosity"));
                                ui.horizontal(|ui| {
                                    for ft in [FluidType::Water, FluidType::Slime, FluidType::Lava, FluidType::Acid] {
                                        if ui.selectable_label(editor.fluid_type == ft, ft.name()).clicked() {
                                            editor.fluid_type = ft;
                                        }
                                    }
                                });
                            }
                            EditTool::SoftBridge => {
                                ui.add(egui::Slider::new(&mut editor.bridge_segments, 3..=20).text("Segments"));
                                ui.add(egui::Slider::new(&mut editor.bridge_softness, 0.005..=0.1).text("Softness"));
                            }
                            EditTool::Target => {
                                ui.add(egui::Slider::new(&mut editor.target_width, 0.5..=3.0).text("Width"));
                                ui.add(egui::Slider::new(&mut editor.target_height, 0.1..=1.0).text("Height"));
                                ui.add(egui::Slider::new(&mut editor.target_score, 50..=2000).text("Score"));
                            }
                            EditTool::Drain => {
                                ui.add(egui::Slider::new(&mut editor.drain_width, 1.0..=10.0).text("Width"));
                            }
                            _ => { ui.label("Click on board to place element."); }
                        }
                        if let Some(sel_id) = editor.selected_id {
                            ui.separator();
                            ui.label(format!("Selected: #{}", sel_id));
                            if let Some(elem) = model.elements.iter_mut().find(|e| e.id == sel_id) {
                                ui.add(egui::Slider::new(&mut elem.rotation, -std::f32::consts::PI..=std::f32::consts::PI).text("Rotation"));
                            }
                            if ui.button("Delete Selected").clicked() {
                                delete_selected = Some(sel_id);
                            }
                        }
                        ui.separator();
                        ui.label("Board Management:");
                        ui.horizontal(|ui| { ui.label("Name:"); ui.text_edit_singleline(board_name); });
                        if ui.button("Save Board").clicked() { do_save = true; }
                        if ui.button("Load Board").clicked() { do_load = true; }
                        if ui.button("Clear All").clicked() { do_clear = true; }
                        ui.separator();
                        ui.label("View:");
                        ui.checkbox(&mut settings.show_grid, "Show Grid");
                        ui.checkbox(&mut settings.show_contacts, "Show Contacts");
                        ui.add(egui::Slider::new(&mut settings.scale, 10.0..=50.0).text("Zoom"));
                        ui.horizontal(|ui| { ui.label("X:"); ui.add(egui::Slider::new(&mut settings.cam_x, -20.0..=20.0)); });
                        ui.horizontal(|ui| { ui.label("Y:"); ui.add(egui::Slider::new(&mut settings.cam_y, -5.0..=25.0)); });
                        ui.separator();
                        if ui.button("▶ Play Test  [TAB]").clicked() { switch_to_play = true; }
                    });
            }

            if let Some(sel_id) = delete_selected {
                model.elements.retain(|e| e.id != sel_id);
                model.editor.selected_id = None;
            }
            if do_save { model.save_board_flag = true; }
            if do_load { model.load_board_flag = true; }
            if do_clear { model.elements.clear(); model.editor.selected_id = None; }
            if switch_to_play { model.mode = GameMode::Play; enter_play_mode(model); }
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
                        ui.label(egui::RichText::new(format!("SCORE: {}", play.score)).size(18.0).color(egui::Color32::YELLOW));
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
                            if ui.button("🔄 Restart").clicked() { do_restart = true; }
                        }
                        ui.separator();
                        ui.label("View:");
                        ui.add(egui::Slider::new(&mut settings.scale, 10.0..=50.0).text("Zoom"));
                        ui.checkbox(&mut settings.show_contacts, "Show Contacts");
                        ui.separator();
                        if ui.button("✏ Edit Mode  [TAB]").clicked() { switch_to_edit = true; }
                    });
            }

            if do_restart { enter_play_mode(model); }
            if switch_to_edit { model.mode = GameMode::Editor; enter_editor_mode(model); }
        }
    }
}

fn apply_fluid_drag(model: &mut Model) {
    if let Some(ball_id) = model.play.ball_body_id {
        let ball_body_opt = model
            .world
            .bodies
            .iter()
            .find(|b| b.borrow().id == ball_id);
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
                        let body_opt = model
                            .world
                            .bodies
                            .iter()
                            .find(|b| b.borrow().id == ball_id);
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
    // Left Flipper setup: pivot at (-3.2, -0.6), base resting rot = -0.40, up target = 0.50
    let min_rot_l = -0.40f32;
    let max_rot_l = 0.50f32;

    if let Some(left_id) = model.play.flipper_left_id {
        if let Some(body_ref) = model.world.bodies.iter().find(|b| b.borrow().id == left_id) {
            let mut body = body_ref.borrow_mut();
            // Damping to prevent oscillation
            body.angular_velocity *= 0.92;
            if model.play.flipper_left_active {
                if body.rotation < max_rot_l {
                    body.angular_velocity += 28.0;
                }
            } else {
                if body.rotation > min_rot_l {
                    body.angular_velocity -= 22.0;
                }
            }
            body.rotation = body.rotation.clamp(min_rot_l, max_rot_l);
            // Stop exactly at limits
            if (body.rotation <= min_rot_l && body.angular_velocity < 0.0)
                || (body.rotation >= max_rot_l && body.angular_velocity > 0.0) {
                body.angular_velocity = 0.0;
            }
        }
    }

    // Right Flipper setup: pivot at (1.6, -0.6), base resting rot = 0.40, up target = -0.50
    let min_rot_r = -0.50f32;
    let max_rot_r = 0.40f32;

    if let Some(right_id) = model.play.flipper_right_id {
        if let Some(body_ref) = model.world.bodies.iter().find(|b| b.borrow().id == right_id) {
            let mut body = body_ref.borrow_mut();
            body.angular_velocity *= 0.92;
            if model.play.flipper_right_active {
                if body.rotation > min_rot_r {
                    body.angular_velocity -= 28.0;
                }
            } else {
                if body.rotation < max_rot_r {
                    body.angular_velocity += 22.0;
                }
            }
            body.rotation = body.rotation.clamp(min_rot_r, max_rot_r);
            if (body.rotation <= min_rot_r && body.angular_velocity < 0.0)
                || (body.rotation >= max_rot_r && body.angular_velocity > 0.0) {
                body.angular_velocity = 0.0;
            }
        }
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
        let pos = model.mouse_world;

        match model.editor.tool {
            EditTool::Select => {
                let clicked_id = model.elements.iter().rev().find_map(|e| {
                    let hit = match &e.kind {
                        ElementKind::Wall { width, height } => {
                            let dx = (pos.x - e.position.x).abs();
                            let dy = (pos.y - e.position.y).abs();
                            dx < width * 0.5 + 0.3 && dy < height * 0.5 + 0.3
                        }
                        ElementKind::Bumper { radius, .. } => {
                            (pos - e.position).length() < *radius + 0.3
                        }
                        _ => {
                            let dx = (pos.x - e.position.x).abs();
                            let dy = (pos.y - e.position.y).abs();
                            dx < 1.5 && dy < 1.5
                        }
                    };
                    if hit { Some(e.id) } else { None }
                });
                model.editor.selected_id = clicked_id;
                model.editor.drag_start = Some(pos);
                model.editor.dragging = clicked_id.is_some();
            }
            EditTool::Delete => {
                let clicked_id = model.elements.iter().rev().find_map(|e| {
                    let hit = match &e.kind {
                        ElementKind::Bumper { radius, .. } => {
                            (pos - e.position).length() < *radius + 0.3
                        }
                        ElementKind::FluidPool { radius, .. } => {
                            (pos - e.position).length() < *radius + 0.3
                        }
                        _ => {
                            let dx = (pos.x - e.position.x).abs();
                            let dy = (pos.y - e.position.y).abs();
                            dx < 1.5 && dy < 1.5
                        }
                    };
                    if hit { Some(e.id) } else { None }
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
                    EditTool::Drain => ElementKind::Drain {
                        width: drain_width,
                    },
                    EditTool::BallSpawn => ElementKind::BallSpawn,
                    _ => return,
                };

                let rotation = match model.editor.tool {
                    EditTool::FlipperLeft => -0.45,
                    EditTool::FlipperRight => 0.45,
                    EditTool::Target => 0.0,
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
        model.editor.drag_start = None;
    }
}

fn mouse_moved(_app: &App, model: &mut Model, _pos: Point2) {
    if model.mode == GameMode::Editor && model.editor.dragging {
        if let Some(sel_id) = model.editor.selected_id {
            if let Some(elem) = model.elements.iter_mut().find(|e| e.id == sel_id) {
                let delta = model.mouse_world - model.editor.drag_start.unwrap_or(model.mouse_world);
                elem.position = elem.position + delta;
                model.editor.drag_start = Some(model.mouse_world);
            }
        }
    }
}

fn key_pressed(_app: &App, model: &mut Model, key: Key) {
    match key {
        Key::Tab => {
            match model.mode {
                GameMode::Editor => {
                    model.mode = GameMode::Play;
                    enter_play_mode(model);
                }
                GameMode::Play => {
                    model.mode = GameMode::Editor;
                    enter_editor_mode(model);
                }
            }
        }
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
        draw_grid(&draw);
    }

    if model.mode == GameMode::Editor {
        draw_editor_elements(model, &draw);
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
                        .x_y(body.position.x + body.radius * 0.3, body.position.y + body.radius * 0.3)
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
            .map(|b| {
                Metaball::new(
                    Vec2::new(b.position.x, b.position.y),
                    b.radius,
                    1.0,
                )
            })
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

fn draw_grid(draw: &nannou::Draw) {
    let x_range = -15.0f32..15.0f32;
    let y_range = -5.0f32..25.0f32;

    for x in (x_range.start as i32)..=(x_range.end as i32) {
        draw.line()
            .start(pt2(x as f32, y_range.start))
            .end(pt2(x as f32, y_range.end))
            .weight(0.01)
            .color(rgba(1.0, 1.0, 1.0, 0.06));
    }
    for y in (y_range.start as i32)..=(y_range.end as i32) {
        draw.line()
            .start(pt2(x_range.start, y as f32))
            .end(pt2(x_range.end, y as f32))
            .weight(0.01)
            .color(rgba(1.0, 1.0, 1.0, 0.06));
    }
}

fn draw_editor_elements(model: &Model, draw: &nannou::Draw) {
    for elem in &model.elements {
        let c = elem.color;
        let col = rgb(c[0], c[1], c[2]);

        match &elem.kind {
            ElementKind::Wall { width, height } => {
                draw.rect()
                    .x_y(elem.position.x, elem.position.y)
                    .w_h(*width, *height)
                    .rotate(elem.rotation)
                    .color(col)
                    .stroke(WHITE)
                    .stroke_weight(0.03);
            }
            ElementKind::Bumper { radius, .. } => {
                draw.ellipse()
                    .x_y(elem.position.x, elem.position.y)
                    .w_h(*radius * 2.0, *radius * 2.0)
                    .color(col)
                    .stroke(WHITE)
                    .stroke_weight(0.03);
                draw.ellipse()
                    .x_y(elem.position.x, elem.position.y)
                    .w_h(*radius * 1.2, *radius * 1.2)
                    .color(rgba(1.0, 1.0, 1.0, 0.3));
            }
            ElementKind::Flipper { side, length } => {
                let offset_x = match side {
                    FlipperSide::Left => *length * 0.45,
                    FlipperSide::Right => -*length * 0.45,
                };
                let angle = elem.rotation;
                let cos_a = angle.cos();
                let sin_a = angle.sin();
                let cx = elem.position.x + offset_x * cos_a - 0.0 * sin_a;
                let cy = elem.position.y + offset_x * sin_a + 0.0 * cos_a;
                draw.rect()
                    .x_y(cx, cy)
                    .w_h(*length, FLIPPER_WIDTH)
                    .rotate(elem.rotation)
                    .color(col)
                    .stroke(WHITE)
                    .stroke_weight(0.03);
                // Pivot point
                draw.ellipse()
                    .x_y(elem.position.x, elem.position.y)
                    .radius(0.15)
                    .color(WHITE);
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
                        .stroke(WHITE)
                        .stroke_weight(0.02);
                }
                draw.ellipse()
                    .x_y(elem.position.x, elem.position.y)
                    .radius(0.15)
                    .color(WHITE);
            }
            ElementKind::FluidPool { radius, fluid, .. } => {
                let fluid_color = match fluid {
                    FluidType::Water => rgba(0.2, 0.5, 0.9, 0.4),
                    FluidType::Slime => rgba(0.3, 0.8, 0.2, 0.4),
                    FluidType::Lava => rgba(0.9, 0.3, 0.1, 0.4),
                    FluidType::Acid => rgba(0.7, 0.9, 0.1, 0.4),
                };
                draw.ellipse()
                    .x_y(elem.position.x, elem.position.y)
                    .w_h(*radius * 2.0, *radius * 2.0)
                    .color(fluid_color)
                    .stroke(WHITE)
                    .stroke_weight(0.03);
                draw.ellipse()
                    .x_y(elem.position.x, elem.position.y)
                    .w_h(*radius * 0.6, *radius * 0.6)
                    .color(rgba(1.0, 1.0, 1.0, 0.2));
            }
            ElementKind::SoftBridge {
                end_x,
                segments,
                ..
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
                    .color(WHITE);
                draw.ellipse()
                    .x_y(end.x, end.y)
                    .radius(0.12)
                    .color(WHITE);
                for i in 0..*segments {
                    let t = i as f32 / *segments as f32;
                    let x = start.x + t * (end.x - start.x);
                    draw.line()
                        .start(pt2(x, start.y - 0.15))
                        .end(pt2(x, start.y + 0.15))
                        .weight(0.04)
                        .color(rgba(1.0, 1.0, 1.0, 0.4));
                }
            }
            ElementKind::Target {
                width,
                height,
                score,
            } => {
                draw.rect()
                    .x_y(elem.position.x, elem.position.y)
                    .w_h(*width, *height)
                    .rotate(elem.rotation)
                    .color(col)
                    .stroke(WHITE)
                    .stroke_weight(0.03);
                draw.text(&score.to_string())
                    .x_y(elem.position.x, elem.position.y)
                    .color(WHITE)
                    .font_size(10);
            }
            ElementKind::Drain { width } => {
                draw.rect()
                    .x_y(elem.position.x, elem.position.y)
                    .w_h(*width, 0.3)
                    .color(rgba(0.8, 0.1, 0.1, 0.5))
                    .stroke(RED)
                    .stroke_weight(0.05);
            }
            ElementKind::BallSpawn => {
                draw.ellipse()
                    .x_y(elem.position.x, elem.position.y)
                    .radius(BALL_RADIUS)
                    .color(rgba(1.0, 1.0, 1.0, 0.5))
                    .stroke(WHITE)
                    .stroke_weight(0.03);
                draw.text("S")
                    .x_y(elem.position.x, elem.position.y)
                    .color(WHITE)
                    .font_size(10);
            }
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
        }
    }
}

