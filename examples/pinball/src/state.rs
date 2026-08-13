use nannou::prelude::*;
use nannou_egui::Egui;
use std::collections::HashMap;
use sylt_2d::math_utils::Vec2;
use sylt_2d::world::World;

pub(crate) const ITERATIONS: u32 = 120;
pub(crate) const BALL_RADIUS: f32 = 0.35;
pub(crate) const FLIPPER_LENGTH: f32 = 2.2;
pub(crate) const FLIPPER_WIDTH: f32 = 0.45;
pub(crate) const FLIPPER_UP_DELTA: f32 = 0.9;
pub(crate) const PLUNGER_MAX_CHARGE: f32 = 55.0;
pub(crate) const GRID_SNAP: f32 = 0.25;
pub(crate) const BALL_MAX_SPEED: f32 = 60.0;
pub(crate) const BUMPER_COOLDOWN: f32 = 0.3;
pub(crate) const TARGET_COOLDOWN: f32 = 0.5;
pub(crate) const COMBO_WINDOW: f32 = 3.0;
pub(crate) const BALL_SAVE_DURATION: f32 = 2.0;
pub(crate) const BALL_TRAIL_LENGTH: usize = 12;

#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) enum EditTool {
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
    pub(crate) fn name(&self) -> &str {
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
pub(crate) enum FluidType {
    Water,
    Slime,
    Lava,
    Acid,
}

impl FluidType {
    pub(crate) fn name(&self) -> &str {
        match self {
            FluidType::Water => "Water",
            FluidType::Slime => "Slime",
            FluidType::Lava => "Lava",
            FluidType::Acid => "Acid",
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) enum GameMode {
    Editor,
    Play,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) enum FlipperSide {
    Left,
    Right,
}

#[derive(Clone, Debug)]
pub(crate) enum ElementKind {
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
pub(crate) struct BoardElement {
    pub(crate) id: usize,
    pub(crate) position: Vec2,
    pub(crate) rotation: f32,
    pub(crate) kind: ElementKind,
    pub(crate) color: [f32; 3],
}

pub(crate) struct EditorState {
    pub(crate) tool: EditTool,
    pub(crate) selected_id: Option<usize>,
    pub(crate) drag_start: Option<Vec2>,
    pub(crate) dragging: bool,
    pub(crate) rotating: bool,
    pub(crate) wall_width: f32,
    pub(crate) wall_height: f32,
    pub(crate) bumper_radius: f32,
    pub(crate) bumper_boost: f32,
    pub(crate) bumper_score: u32,
    pub(crate) chain_links: usize,
    pub(crate) chain_length: f32,
    pub(crate) chain_end_mass: f32,
    pub(crate) fluid_radius: f32,
    pub(crate) fluid_type: FluidType,
    pub(crate) fluid_viscosity: f32,
    pub(crate) bridge_segments: usize,
    pub(crate) bridge_softness: f32,
    pub(crate) target_width: f32,
    pub(crate) target_height: f32,
    pub(crate) target_score: u32,
    pub(crate) drain_width: f32,
    pub(crate) next_color_idx: usize,
}

pub(crate) struct PlayState {
    pub(crate) score: u32,
    pub(crate) balls_remaining: u32,
    pub(crate) ball_body_id: Option<usize>,
    pub(crate) flipper_left_id: Option<usize>,
    pub(crate) flipper_right_id: Option<usize>,
    pub(crate) flipper_left_rest: f32,
    pub(crate) flipper_left_up: f32,
    pub(crate) flipper_right_rest: f32,
    pub(crate) flipper_right_up: f32,
    pub(crate) plunger_body_id: Option<usize>,
    pub(crate) plunger_charge: f32,
    pub(crate) plunger_charging: bool,
    pub(crate) flipper_left_active: bool,
    pub(crate) flipper_right_active: bool,
    pub(crate) chain_bodies: Vec<usize>,
    pub(crate) chain_anchor_id: Option<usize>,
    pub(crate) metaball_bodies: Vec<usize>,
    pub(crate) metaball_colors: HashMap<usize, [f32; 3]>,
    pub(crate) metaball_fluid_types: HashMap<usize, FluidType>,
    pub(crate) fluid_particle_ids: Vec<usize>,
    pub(crate) fluid_time: f32,
    pub(crate) fluid_drag_active: bool,
    pub(crate) game_over: bool,
    pub(crate) high_score: u32,
    pub(crate) bumper_flash_timers: Vec<(Vec2, f32)>,
    pub(crate) ball_trail: Vec<Vec2>,
    pub(crate) bumper_cooldowns: Vec<(Vec2, f32)>,
    pub(crate) target_cooldowns: Vec<(usize, f32)>,
    pub(crate) combo_multiplier: u32,
    pub(crate) combo_timer: f32,
    pub(crate) ball_save_timer: f32,
    pub(crate) score_popups: Vec<(Vec2, u32, f32)>,
    pub(crate) bumper_particles: Vec<(Vec2, Vec2, f32)>,
}

pub(crate) struct EguiSettings {
    pub(crate) scale: f32,
    pub(crate) cam_x: f32,
    pub(crate) cam_y: f32,
    pub(crate) show_grid: bool,
    pub(crate) snap_to_grid: bool,
    pub(crate) grid_size: f32,
    pub(crate) show_contacts: bool,
    pub(crate) metaball_resolution: usize,
    pub(crate) metaball_threshold: f32,
}

pub(crate) struct DemoState {
    pub(crate) active: bool,
    pub(crate) timer: f32,
    pub(crate) phase: DemoPhase,
    pub(crate) editor_tool_idx: usize,
    pub(crate) caption: String,
}

#[derive(Clone, Copy, PartialEq)]
pub(crate) enum DemoPhase {
    Init,
    BuildBoard,
    SwitchToPlay,
    WaitAfterPlay,
    ChargePlunger,
    LaunchBall,
    BallInPlay,
    WatchBall,
    EditorTools,
    Done,
}

pub(crate) struct Model {
    pub(crate) _window: window::Id,
    pub(crate) time_step: f32,
    pub(crate) world: World,
    pub(crate) egui: Egui,
    pub(crate) settings: EguiSettings,
    pub(crate) mode: GameMode,
    pub(crate) elements: Vec<BoardElement>,
    pub(crate) editor: EditorState,
    pub(crate) play: PlayState,
    pub(crate) demo: DemoState,
    pub(crate) next_id: usize,
    pub(crate) board_name_input: String,
    pub(crate) load_board_flag: bool,
    pub(crate) save_board_flag: bool,
    pub(crate) mouse_world: Vec2,
    pub(crate) is_first_frame: bool,
}
