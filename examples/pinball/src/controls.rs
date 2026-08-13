use crate::board::{enter_editor_mode, enter_play_mode};
use crate::physics::launch_plunger;
use crate::state::{
    BoardElement, EditTool, ElementKind, FlipperSide, GameMode, Model, FLIPPER_LENGTH,
};
use crate::util::{element_hit, rotation_handle_pos, snap_to_grid};
use nannou::prelude::*;

pub(crate) fn raw_window_event(
    _app: &App,
    model: &mut Model,
    event: &nannou::winit::event::WindowEvent,
) {
    model.egui.handle_raw_event(event);
}

pub(crate) fn mouse_pressed(_app: &App, model: &mut Model, button: MouseButton) {
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

pub(crate) fn mouse_released(_app: &App, model: &mut Model, _button: MouseButton) {
    if model.mode == GameMode::Editor {
        model.editor.dragging = false;
        model.editor.rotating = false;
        model.editor.drag_start = None;
    }
}

pub(crate) fn mouse_moved(_app: &App, model: &mut Model, _pos: Point2) {
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
                    sylt_2d::math_utils::Vec2::new(
                        crate::util::snap_value(new_pos.x, model.settings.grid_size),
                        crate::util::snap_value(new_pos.y, model.settings.grid_size),
                    )
                } else {
                    new_pos
                };
                model.editor.drag_start = Some(model.mouse_world);
            }
        }
    }
}

pub(crate) fn key_pressed(_app: &App, model: &mut Model, key: Key) {
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

pub(crate) fn key_released(_app: &App, model: &mut Model, key: Key) {
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
