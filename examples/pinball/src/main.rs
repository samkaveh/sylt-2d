mod board;
mod controls;
mod demo;
mod physics;
mod render;
mod state;
mod ui;
mod util;

use crate::board::enter_play_mode;
use crate::demo::update_demo;
use crate::physics::{
    animate_fluid_particles, apply_flippers, apply_fluid_drag, apply_plunger, detect_scoring,
};
use crate::render::view;
use crate::state::{DemoPhase, EditTool, EguiSettings, FluidType, GameMode, Model, PlayState};
use crate::ui::{draw_editor_panel, draw_play_panel};
use crate::util::screen_to_world;
use nannou::prelude::*;
use nannou_egui::Egui;
use sylt_2d::math_utils::Vec2;
use sylt_2d::world::World;

fn main() {
    nannou::app(model).update(update).run();
}

fn model(app: &App) -> Model {
    let _window = app
        .new_window()
        .view(view)
        .raw_event(controls::raw_window_event)
        .key_pressed(controls::key_pressed)
        .key_released(controls::key_released)
        .mouse_pressed(controls::mouse_pressed)
        .mouse_released(controls::mouse_released)
        .mouse_moved(controls::mouse_moved)
        .build()
        .unwrap();
    let window = app.window(_window).unwrap();
    let egui = Egui::from_window(&window);
    let mut world = World::new(Vec2::new(0.0, -15.0), state::ITERATIONS);
    // Give the ball a solid bounce off walls/flippers so an in-plane hit keeps
    // its speed instead of dead-stopping (this is what made the ball feel slow
    // after touching a flipper).
    world.set_restitution(0.35);

    let editor = state::EditorState {
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
        metaball_colors: std::collections::HashMap::new(),
        metaball_fluid_types: std::collections::HashMap::new(),
        fluid_particle_ids: Vec::new(),
        fluid_time: 0.0,
        fluid_drag_active: false,
        game_over: false,
        high_score: 0,
        bumper_flash_timers: Vec::new(),
        ball_trail: Vec::new(),
        bumper_cooldowns: Vec::new(),
        target_cooldowns: Vec::new(),
        combo_multiplier: 1,
        combo_timer: 0.0,
        ball_save_timer: 0.0,
        score_popups: Vec::new(),
        bumper_particles: Vec::new(),
    };

    let demo_enabled = std::env::args().any(|a| a == "--demo");

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
            grid_size: state::GRID_SNAP,
            show_contacts: false,
            metaball_resolution: 40,
            metaball_threshold: 0.5,
        },
        mode: GameMode::Editor,
        elements: Vec::new(),
        editor,
        play,
        demo: state::DemoState {
            active: demo_enabled,
            timer: 0.0,
            phase: DemoPhase::Init,
            editor_tool_idx: 0,
            caption: String::new(),
        },
        next_id: 1,
        board_name_input: String::from("my_board"),
        load_board_flag: false,
        save_board_flag: false,
        mouse_world: Vec2::new(0.0, 0.0),
        is_first_frame: true,
    }
}

fn update(app: &App, model: &mut Model, _update: Update) {
    model.mouse_world = screen_to_world(app, &model.settings);

    if model.is_first_frame {
        model.is_first_frame = false;
        if !model.demo.active {
            enter_play_mode(model);
            model.mode = GameMode::Play;
        }
    }

    if model.demo.active {
        update_demo(model);
    }

    if model.mode == GameMode::Play && !model.play.game_over {
        apply_flippers(model);
        apply_plunger(model);
        let _ = model.world.step(model.time_step);
        apply_fluid_drag(model);
        animate_fluid_particles(model, model.time_step);
        detect_scoring(model);

        // Update ball trail
        if let Some(ball_id) = model.play.ball_body_id {
            if let Some(body_ref) = model.world.bodies.iter().find(|b| b.borrow().id == ball_id) {
                let pos = body_ref.borrow().position;
                model.play.ball_trail.push(pos);
                if model.play.ball_trail.len() > state::BALL_TRAIL_LENGTH {
                    model.play.ball_trail.remove(0);
                }
            }
        }

        // Update ball save timer
        if model.play.ball_save_timer > 0.0 {
            model.play.ball_save_timer -= model.time_step;
        }

        // Update visual flash effects
        model.play.bumper_flash_timers.retain_mut(|(_, timer)| {
            *timer -= model.time_step;
            *timer > 0.0
        });

        // Update bumper particles
        model.play.bumper_particles.retain_mut(|(_, _, life)| {
            *life -= model.time_step;
            *life > 0.0
        });
    }

    model.egui.set_elapsed_time(_update.since_start);

    match model.mode {
        GameMode::Editor => {
            let actions = draw_editor_panel(model);
            if let Some(sel_id) = actions.delete_selected {
                model.elements.retain(|e| e.id != sel_id);
                model.editor.selected_id = None;
            }
            if actions.do_save {
                model.save_board_flag = true;
            }
            if actions.do_load {
                model.load_board_flag = true;
            }
            if actions.do_clear {
                model.elements.clear();
                model.editor.selected_id = None;
            }
            if actions.switch_to_play {
                model.mode = GameMode::Play;
                enter_play_mode(model);
            }
        }
        GameMode::Play => {
            let actions = draw_play_panel(model);
            if actions.do_restart {
                enter_play_mode(model);
            }
            if actions.switch_to_edit {
                model.mode = GameMode::Editor;
                board::enter_editor_mode(model);
            }
        }
    }
}
