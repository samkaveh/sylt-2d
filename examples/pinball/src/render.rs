use crate::state::{
    BoardElement, EditTool, FlipperSide, GameMode, Model, BALL_RADIUS, BALL_TRAIL_LENGTH,
    COMBO_WINDOW, FLIPPER_UP_DELTA, PLUNGER_MAX_CHARGE,
};
use crate::util::{rotate_vec, snap_to_grid};
use nannou::prelude::*;
use sylt_2d::body::Shape;
use sylt_2d::math_utils::Vec2;
use sylt_2d::metaball::{
    cluster_metaballs, compute_metaball_bounds, marching_squares_debug, Metaball,
};

pub(crate) type Label = (Vec2, String, f32, [f32; 3], f32);

pub(crate) fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    // Text labels are rasterised in screen space and presented after the world
    // render in one extra pass, so glyphs stay crisp at any zoom.
    let mut labels: Vec<Label> = Vec::new();
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

    if model.settings.show_grid && model.mode == GameMode::Editor {
        draw_grid(&draw, &model.settings);
    }

    if model.mode == GameMode::Editor {
        draw_editor_elements(model, &draw, &mut labels);
        draw_placement_preview(model, &draw, &mut labels);

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
            &mut labels,
            &model.settings,
            model.mouse_world + Vec2::new(0.6, 0.5),
            format!("({:.2}, {:.2})", snapped.x, snapped.y),
            13.0,
            [1.0, 1.0, 0.6],
            0.9,
        );
    }

    // Ball trail / motion blur (behind all bodies)
    if model.mode == GameMode::Play && model.play.ball_trail.len() > 1 {
        for (i, pos) in model.play.ball_trail.iter().enumerate() {
            let t = i as f32 / BALL_TRAIL_LENGTH as f32;
            let alpha = (1.0 - t) * 0.25;
            let size = BALL_RADIUS * 2.0 * (1.0 - t * 0.4);
            if alpha > 0.01 {
                draw.ellipse()
                    .x_y(pos.x, pos.y)
                    .w_h(size, size)
                    .color(rgba(0.4, 0.6, 0.9, alpha));
            }
        }
    }

    // Bumper hit particles (behind all bodies)
    for (pos, vel, life) in &model.play.bumper_particles {
        let t = 1.0 - (life / 0.3);
        let alpha = (1.0 - t) * 0.9;
        let size = 0.12 * (1.0 - t * 0.5);
        if alpha > 0.01 {
            draw.ellipse()
                .x_y(pos.x + vel.x * t, pos.y + vel.y * t)
                .w_h(size, size)
                .color(rgba(1.0, 0.9, 0.4, alpha));
        }
    }

    // Render fluid pool isosurfaces BEHIND the pinball, flippers, and walls
    if !model.play.metaball_bodies.is_empty() {
        let metaballs: Vec<Metaball> = model
            .world
            .iter_bodies()
            .filter(|b| model.play.metaball_bodies.contains(&b.id))
            .map(|b| Metaball::new(Vec2::new(b.position.x, b.position.y), b.radius, 1.0))
            .collect();

        if !metaballs.is_empty() {
            let clusters = cluster_metaballs(&metaballs, model.settings.metaball_threshold);
            let time = model.play.fluid_time;

            for cluster in &clusters {
                let (bounds_min, bounds_max) = compute_metaball_bounds(
                    &cluster.metaballs,
                    model.settings.metaball_threshold,
                    2.0,
                );

                let (cluster_color, fluid_type) = cluster
                    .metaballs
                    .iter()
                    .find_map(|m| {
                        model
                            .world
                            .iter_bodies()
                            .find(|b| {
                                model.play.metaball_bodies.contains(&b.id)
                                    && (b.position - m.position).length() < 0.01
                            })
                            .and_then(|b| {
                                let c = model.play.metaball_colors.get(&b.id).copied()?;
                                let ft = model
                                    .play
                                    .metaball_fluid_types
                                    .get(&b.id)
                                    .copied()
                                    .unwrap_or(crate::state::FluidType::Water);
                                Some((c, ft))
                            })
                    })
                    .unwrap_or(([0.2, 0.6, 0.95], crate::state::FluidType::Water));

                // Primary fluid surface contour (outer boundary)
                let primary_ms = marching_squares_debug(
                    &cluster.metaballs,
                    bounds_min,
                    bounds_max,
                    model.settings.metaball_resolution,
                    model.settings.metaball_threshold,
                    &[],
                );

                // Secondary inner contour (core depth layer)
                let inner_threshold = model.settings.metaball_threshold * 1.8;
                let inner_ms = marching_squares_debug(
                    &cluster.metaballs,
                    bounds_min,
                    bounds_max,
                    model.settings.metaball_resolution,
                    inner_threshold,
                    &[],
                );

                let (base_alpha, stroke_color, inner_alpha, glow_color) = match fluid_type {
                    crate::state::FluidType::Water => (
                        0.65,
                        rgba(0.7, 0.9, 1.0, 0.9),
                        0.35,
                        rgba(0.1, 0.4, 0.8, 0.4),
                    ),
                    crate::state::FluidType::Slime => (
                        0.80,
                        rgba(0.6, 1.0, 0.4, 0.95),
                        0.45,
                        rgba(0.2, 0.6, 0.1, 0.5),
                    ),
                    crate::state::FluidType::Lava => (
                        0.85,
                        rgba(1.0, 0.8, 0.2, 0.95),
                        0.55,
                        rgba(1.0, 0.2, 0.0, 0.6),
                    ),
                    crate::state::FluidType::Acid => (
                        0.75,
                        rgba(0.9, 1.0, 0.3, 0.95),
                        0.40,
                        rgba(0.5, 0.8, 0.0, 0.5),
                    ),
                };

                // 1. Ambient outer glow / refraction halo
                for poly in &primary_ms.polygons {
                    if poly.len() >= 3 {
                        draw.polygon()
                            .x_y(0.0, 0.0)
                            .color(rgba(
                                cluster_color[0],
                                cluster_color[1],
                                cluster_color[2],
                                base_alpha * 0.4,
                            ))
                            .stroke(glow_color)
                            .stroke_weight(0.12)
                            .points(poly.clone());
                    }
                }

                // 2. Base liquid body fill with crisp boundary stroke
                for poly in &primary_ms.polygons {
                    if poly.len() >= 3 {
                        draw.polygon()
                            .x_y(0.0, 0.0)
                            .color(rgba(
                                cluster_color[0],
                                cluster_color[1],
                                cluster_color[2],
                                base_alpha,
                            ))
                            .stroke(stroke_color)
                            .stroke_weight(0.05)
                            .points(poly.clone());
                    }
                }

                // 3. Inner dense fluid core (depth effect)
                for poly in &inner_ms.polygons {
                    if poly.len() >= 3 {
                        draw.polygon()
                            .x_y(0.0, 0.0)
                            .color(rgba(
                                (cluster_color[0] * 1.2).min(1.0),
                                (cluster_color[1] * 1.2).min(1.0),
                                (cluster_color[2] * 1.2).min(1.0),
                                inner_alpha,
                            ))
                            .points(poly.clone());
                    }
                }

                // 4. Liquid surface specular glints and bubbles
                for mb in &cluster.metaballs {
                    match fluid_type {
                        crate::state::FluidType::Water => {
                            let shimmer = (time * 3.0 + mb.position.x * 2.0).sin() * 0.03;
                            draw.ellipse()
                                .x_y(mb.position.x - 0.05, mb.position.y + 0.08)
                                .w_h(0.14 + shimmer, 0.07)
                                .color(rgba(1.0, 1.0, 1.0, 0.45));
                        }
                        crate::state::FluidType::Lava => {
                            let pulse = (time * 4.0 + mb.position.x * 1.5).sin().abs() * 0.1;
                            draw.ellipse()
                                .x_y(mb.position.x, mb.position.y)
                                .w_h(0.2 + pulse, 0.2 + pulse)
                                .color(rgba(1.0, 0.9, 0.3, 0.6));
                        }
                        crate::state::FluidType::Acid => {
                            let bubble = (time * 5.0 + mb.position.y * 3.0).sin();
                            if bubble > 0.4 {
                                draw.ellipse()
                                    .x_y(
                                        mb.position.x + 0.04 * bubble,
                                        mb.position.y + 0.06 * bubble,
                                    )
                                    .w_h(0.09, 0.09)
                                    .color(rgba(0.9, 1.0, 0.5, 0.7));
                            }
                        }
                        crate::state::FluidType::Slime => {
                            draw.ellipse()
                                .x_y(mb.position.x - 0.04, mb.position.y + 0.06)
                                .w_h(0.12, 0.08)
                                .color(rgba(0.8, 1.0, 0.6, 0.35));
                        }
                    }
                }
            }
        }
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
                    // Metallic Plunger with charge meter
                    let charge_pct = model.play.plunger_charge / PLUNGER_MAX_CHARGE;
                    let p_color = rgb(0.9, 0.2 + charge_pct * 0.7, 0.1);
                    let p_stroke = rgb(1.0, 0.5 + charge_pct * 0.5, 0.3);
                    draw.rect()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.width.x, body.width.y)
                        .rotate(body.rotation)
                        .color(p_color)
                        .stroke(p_stroke)
                        .stroke_weight(0.05);

                    draw.rect()
                        .x_y(body.position.x - body.width.x * 0.2, body.position.y)
                        .w_h(body.width.x * 0.2, body.width.y * 0.6)
                        .rotate(body.rotation)
                        .color(rgba(1.0, 1.0, 1.0, 0.3));

                    let meter_w = body.width.x * 0.8;
                    let meter_h = 0.12;
                    let meter_x = body.position.x;
                    let meter_y = body.position.y + body.width.y * 0.5 + 0.2;
                    draw.rect()
                        .x_y(meter_x, meter_y)
                        .w_h(meter_w, meter_h)
                        .color(rgb(0.1, 0.1, 0.15))
                        .stroke(rgba(1.0, 1.0, 1.0, 0.2))
                        .stroke_weight(0.02);
                    if charge_pct > 0.01 {
                        let fill_w = meter_w * charge_pct;
                        let fill_color = if charge_pct < 0.5 {
                            rgb(0.2, 0.8, 0.3)
                        } else if charge_pct < 0.8 {
                            rgb(0.9, 0.8, 0.1)
                        } else {
                            rgb(0.95, 0.2, 0.1)
                        };
                        draw.rect()
                            .x_y(meter_x - (meter_w - fill_w) * 0.5, meter_y)
                            .w_h(fill_w, meter_h * 0.7)
                            .color(fill_color);
                    }
                } else if is_flipper_left || is_flipper_right {
                    // Metallic Flipper with tapered shape
                    let f_color = if (is_flipper_left && model.play.flipper_left_active)
                        || (is_flipper_right && model.play.flipper_right_active)
                    {
                        rgb(0.1, 0.95, 0.85) // Bright cyan when active
                    } else {
                        rgb(0.15, 0.7, 0.5) // Emerald resting
                    };
                    let f_stroke = if (is_flipper_left && model.play.flipper_left_active)
                        || (is_flipper_right && model.play.flipper_right_active)
                    {
                        rgb(0.6, 1.0, 0.95)
                    } else {
                        rgb(0.3, 0.85, 0.65)
                    };

                    draw.rect()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.width.x, body.width.y)
                        .rotate(body.rotation)
                        .color(f_color)
                        .stroke(f_stroke)
                        .stroke_weight(0.04);

                    let pivot_offset = if is_flipper_left {
                        -body.width.x * 0.42
                    } else {
                        body.width.x * 0.42
                    };
                    let cos_r = body.rotation.cos();
                    let sin_r = body.rotation.sin();
                    let pivot_x = body.position.x + pivot_offset * cos_r;
                    let pivot_y = body.position.y + pivot_offset * sin_r;
                    draw.ellipse()
                        .x_y(pivot_x, pivot_y)
                        .w_h(0.28, 0.28)
                        .color(rgb(0.7, 0.75, 0.8))
                        .stroke(WHITE)
                        .stroke_weight(0.03);

                    let tip_offset = if is_flipper_left {
                        body.width.x * 0.42
                    } else {
                        -body.width.x * 0.42
                    };
                    let tip_x = body.position.x + tip_offset * cos_r;
                    let tip_y = body.position.y + tip_offset * sin_r;
                    draw.ellipse()
                        .x_y(tip_x, tip_y)
                        .w_h(0.16, 0.16)
                        .color(rgba(1.0, 1.0, 1.0, 0.4));
                } else if is_chain {
                    draw.rect()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.width.x, body.width.y)
                        .rotate(body.rotation)
                        .color(rgb(0.65, 0.65, 0.75))
                        .stroke(rgb(0.9, 0.9, 1.0))
                        .stroke_weight(0.02);
                } else if num == 0 {
                    // Bottom Drain Floor - danger zone
                    draw.rect()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.width.x, body.width.y)
                        .rotate(body.rotation)
                        .color(rgb(0.6, 0.08, 0.12))
                        .stroke(rgb(0.9, 0.15, 0.2))
                        .stroke_weight(0.04);
                    draw.rect()
                        .x_y(body.position.x, body.position.y + body.width.y * 0.3)
                        .w_h(body.width.x * 0.95, body.width.y * 0.3)
                        .color(rgba(1.0, 0.2, 0.15, 0.3));
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
                    .color(rgb(0.18, 0.22, 0.32))
                    .x_y(body.position.x, body.position.y)
                    .rotate(body.rotation)
                    .stroke(rgb(0.35, 0.45, 0.65))
                    .stroke_weight(0.03)
                    .points(tuples);
            }
            Shape::Circle => {
                if is_ball {
                    // Chrome Metallic Pinball with Enhanced Glow
                    draw.ellipse()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.radius * 3.0, body.radius * 3.0)
                        .color(rgba(0.2, 0.5, 1.0, 0.15));

                    draw.ellipse()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.radius * 2.2, body.radius * 2.2)
                        .color(rgba(0.4, 0.6, 0.9, 0.3));

                    draw.ellipse()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.radius * 2.0, body.radius * 2.0)
                        .color(rgb(0.85, 0.88, 0.95))
                        .stroke(rgb(0.95, 0.97, 1.0))
                        .stroke_weight(0.03);

                    draw.ellipse()
                        .x_y(
                            body.position.x + body.radius * 0.2,
                            body.position.y + body.radius * 0.2,
                        )
                        .w_h(body.radius * 0.7, body.radius * 0.7)
                        .color(rgba(1.0, 1.0, 1.0, 0.9));

                    draw.ellipse()
                        .x_y(
                            body.position.x + body.radius * 0.35,
                            body.position.y + body.radius * 0.35,
                        )
                        .w_h(body.radius * 0.3, body.radius * 0.3)
                        .color(WHITE);

                    draw.ellipse()
                        .x_y(
                            body.position.x - body.radius * 0.2,
                            body.position.y - body.radius * 0.15,
                        )
                        .w_h(body.radius * 0.25, body.radius * 0.2)
                        .color(rgba(0.3, 0.4, 0.6, 0.5));
                } else if model.play.metaball_bodies.contains(&body.id) {
                    // Metaball particle bodies are rendered seamlessly as fluid isosurfaces
                    // in the metaball pass below, so we don't draw individual circles here.
                } else if body.inv_mass == 0.0 {
                    // Bumper Rendering with enhanced glow
                    let position = body.position;
                    let flash_timer = model
                        .play
                        .bumper_flash_timers
                        .iter()
                        .find(|(pos, _)| (*pos - position).length() < 0.1)
                        .map(|(_, t)| *t)
                        .unwrap_or(0.0);
                    let is_flashing = flash_timer > 0.0;

                    let (b_col, ring_col, glow_alpha) = if is_flashing {
                        let pulse = (flash_timer * 12.0).sin().abs();
                        (rgb(1.0, 1.0, 0.6), rgb(1.0, 0.9, 0.3), 0.35 + 0.25 * pulse)
                    } else {
                        (rgb(0.95, 0.35, 0.2), rgb(1.0, 0.6, 0.2), 0.15)
                    };

                    draw.ellipse()
                        .x_y(position.x, position.y)
                        .w_h(body.radius * 3.0, body.radius * 3.0)
                        .color(rgba(b_col.red, b_col.green, b_col.blue, glow_alpha * 0.4));

                    draw.ellipse()
                        .x_y(position.x, position.y)
                        .w_h(body.radius * 2.4, body.radius * 2.4)
                        .color(rgba(b_col.red, b_col.green, b_col.blue, glow_alpha));

                    draw.ellipse()
                        .x_y(position.x, position.y)
                        .w_h(body.radius * 2.0, body.radius * 2.0)
                        .color(b_col)
                        .stroke(rgba(1.0, 1.0, 1.0, 0.8))
                        .stroke_weight(0.04);

                    draw.ellipse()
                        .x_y(position.x, position.y)
                        .w_h(body.radius * 1.3, body.radius * 1.3)
                        .color(ring_col);

                    draw.ellipse()
                        .x_y(
                            position.x - body.radius * 0.25,
                            position.y + body.radius * 0.25,
                        )
                        .w_h(body.radius * 0.5, body.radius * 0.5)
                        .color(rgba(1.0, 1.0, 1.0, 0.5));
                } else {
                    draw.ellipse()
                        .x_y(body.position.x, body.position.y)
                        .w_h(body.radius * 2.0, body.radius * 2.0)
                        .color(rgb(0.5, 0.5, 0.6));
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

    // Floating score popups on hits (screen space)
    for (pos, points, timer) in &model.play.score_popups {
        let t = 1.0 - (timer / 0.8);
        let rise = t * 1.2;
        let alpha = (1.0 - t) * 0.95;
        if alpha > 0.01 {
            draw_world_label(
                &mut labels,
                &model.settings,
                *pos + Vec2::new(0.0, rise),
                format!("+{}", points),
                16.0,
                [1.0, 0.95, 0.3],
                alpha,
            );
        }
    }

    // Combo multiplier badge (screen space, top of playfield)
    if model.mode == GameMode::Play && model.play.combo_multiplier > 1 {
        let combo_alpha = (model.play.combo_timer / COMBO_WINDOW).clamp(0.0, 1.0);
        if combo_alpha > 0.01 {
            let cx = -5.0;
            let cy = 21.0;
            let scale = 1.0 + (1.0 - combo_alpha) * 0.15;
            draw_world_label(
                &mut labels,
                &model.settings,
                Vec2::new(cx, cy),
                format!("x{} COMBO", model.play.combo_multiplier),
                20.0 * scale,
                [1.0, 0.6, 0.2],
                combo_alpha,
            );
        }
    }

    draw.to_frame(app, &frame).unwrap();
    // Rasterise the queued labels in screen space on top of the world render.
    if !labels.is_empty() {
        let label_draw = app.draw();
        for (pos, text, px, color, alpha) in &labels {
            label_draw
                .text(text)
                .x_y(pos.x, pos.y)
                .font_size(px.max(8.0) as u32)
                .color(rgba(color[0], color[1], color[2], *alpha))
                .align_text_middle_y()
                .center_justify();
        }
        label_draw.to_frame(app, &frame).unwrap();
    }
    model.egui.draw_to_frame(&frame).unwrap();

    // Demo feature caption, drawn last in screen space so it always reads
    // clearly on top (immune to world-scale font rounding).
    if model.demo.active && !model.demo.caption.is_empty() {
        let draw = app.draw();
        let rect = app.window_rect();
        let banner_w = (rect.w() * 0.9).min(920.0);
        let cy = rect.top() - 44.0;
        draw.rect()
            .x_y(0.0, cy)
            .w_h(banner_w, 52.0)
            .color(rgba(0.05, 0.06, 0.12, 0.92))
            .stroke(rgba(0.35, 0.62, 1.0, 0.95))
            .stroke_weight(2.0);
        draw.text(&model.demo.caption)
            .x_y(0.0, cy)
            .font_size(26)
            .color(rgb(1.0, 1.0, 1.0))
            .align_text_middle_y()
            .center_justify();
        draw.to_frame(app, &frame).unwrap();
    }

    // Game over overlay (screen space, drawn over everything)
    if model.mode == GameMode::Play && model.play.game_over {
        let draw = app.draw();
        let rect = app.window_rect();
        draw.rect()
            .x_y(0.0, 0.0)
            .w_h(rect.w(), rect.h())
            .color(rgba(0.0, 0.0, 0.0, 0.55));
        draw.text("💥 GAME OVER 💥")
            .x_y(0.0, rect.top() - 180.0)
            .font_size(72)
            .color(rgb(1.0, 0.3, 0.2))
            .align_text_middle_y()
            .center_justify();
        draw.text(&format!("FINAL SCORE: {}", model.play.score))
            .x_y(0.0, rect.top() - 260.0)
            .font_size(42)
            .color(rgb(1.0, 1.0, 1.0))
            .align_text_middle_y()
            .center_justify();
        draw.text(&format!("HIGH SCORE: {}", model.play.high_score))
            .x_y(0.0, rect.top() - 320.0)
            .font_size(32)
            .color(rgb(0.6, 0.9, 0.4))
            .align_text_middle_y()
            .center_justify();
        draw.text("Press R to Restart  |  TAB for Editor")
            .x_y(0.0, rect.top() - 400.0)
            .font_size(28)
            .color(rgb(0.8, 0.8, 0.85))
            .align_text_middle_y()
            .center_justify();
        draw.to_frame(app, &frame).unwrap();
    }
}

pub(crate) fn draw_grid(draw: &nannou::Draw, settings: &crate::state::EguiSettings) {
    let step = settings.grid_size.max(0.05);
    let x_range = -15.0f32..15.0f32;
    let y_range = -5.0f32..25.0f32;

    // Minor grid lines at the snap step.
    let mut gx = crate::util::snap_value(x_range.start, step);
    while gx <= x_range.end {
        draw.line()
            .start(pt2(gx, y_range.start))
            .end(pt2(gx, y_range.end))
            .weight(0.008)
            .color(rgba(1.0, 1.0, 1.0, 0.05));
        gx += step;
    }
    let mut gy = crate::util::snap_value(y_range.start, step);
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
/// Project a world point to screen pixel coordinates for the given camera.
pub(crate) fn world_to_screen(settings: &crate::state::EguiSettings, world_pos: Vec2) -> Vec2 {
    Vec2::new(
        (world_pos.x - settings.cam_x) * settings.scale,
        (world_pos.y - settings.cam_y) * settings.scale,
    )
}

/// Queue a small text label anchored to a world position. The label is stored
/// and later rasterised in *screen space* (via a single draw present at the end
/// of the frame) so the glyphs stay crisp regardless of camera zoom.
pub(crate) fn draw_world_label(
    out: &mut Vec<Label>,
    settings: &crate::state::EguiSettings,
    world_pos: Vec2,
    text: String,
    px: f32,
    color: [f32; 3],
    alpha: f32,
) {
    if px <= 0.0 {
        return;
    }
    let pos = world_to_screen(settings, world_pos);
    out.push((pos, text, px, color, alpha));
}

pub(crate) fn draw_element_shape(elem: &BoardElement, draw: &nannou::Draw, alpha: f32) {
    let c = elem.color;
    let col = rgba(c[0], c[1], c[2], alpha);
    let stroke_col = rgba(1.0, 1.0, 1.0, alpha);

    match &elem.kind {
        crate::state::ElementKind::Wall { width, height } => {
            draw.rect()
                .x_y(elem.position.x, elem.position.y)
                .w_h(*width, *height)
                .rotate(elem.rotation)
                .color(col)
                .stroke(stroke_col)
                .stroke_weight(0.03);
        }
        crate::state::ElementKind::Bumper { radius, .. } => {
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
        crate::state::ElementKind::Flipper { side, length } => {
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
                .w_h(*length, crate::state::FLIPPER_WIDTH)
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
        crate::state::ElementKind::Chain {
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
        crate::state::ElementKind::FluidPool { radius, fluid, .. } => {
            let fluid_color = match fluid {
                crate::state::FluidType::Water => rgba(0.2, 0.5, 0.9, 0.4 * alpha),
                crate::state::FluidType::Slime => rgba(0.3, 0.8, 0.2, 0.4 * alpha),
                crate::state::FluidType::Lava => rgba(0.9, 0.3, 0.1, 0.4 * alpha),
                crate::state::FluidType::Acid => rgba(0.7, 0.9, 0.1, 0.4 * alpha),
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
        crate::state::ElementKind::SoftBridge {
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
        crate::state::ElementKind::Target { width, height, .. } => {
            draw.rect()
                .x_y(elem.position.x, elem.position.y)
                .w_h(*width, *height)
                .rotate(elem.rotation)
                .color(col)
                .stroke(stroke_col)
                .stroke_weight(0.03);
        }
        crate::state::ElementKind::Drain { width } => {
            draw.rect()
                .x_y(elem.position.x, elem.position.y)
                .w_h(*width, 0.3)
                .color(rgba(0.8, 0.1, 0.1, 0.5 * alpha))
                .stroke(rgba(1.0, 0.0, 0.0, alpha))
                .stroke_weight(0.05);
        }
        crate::state::ElementKind::BallSpawn => {
            draw.ellipse()
                .x_y(elem.position.x, elem.position.y)
                .radius(BALL_RADIUS)
                .color(rgba(1.0, 1.0, 1.0, 0.5 * alpha))
                .stroke(stroke_col)
                .stroke_weight(0.03);
        }
    }
}

pub(crate) fn draw_editor_elements(model: &Model, draw: &nannou::Draw, labels: &mut Vec<Label>) {
    for elem in &model.elements {
        draw_element_shape(elem, draw, 1.0);

        // Small labels for scoring/identifier elements.
        match &elem.kind {
            crate::state::ElementKind::Target { score, .. } => {
                draw_world_label(
                    labels,
                    &model.settings,
                    elem.position + Vec2::new(0.0, -0.45),
                    score.to_string(),
                    13.0,
                    [1.0, 1.0, 1.0],
                    0.9,
                );
            }
            crate::state::ElementKind::BallSpawn => {
                draw_world_label(
                    labels,
                    &model.settings,
                    elem.position + Vec2::new(0.0, -0.45),
                    "S".to_string(),
                    13.0,
                    [1.0, 1.0, 1.0],
                    0.9,
                );
            }
            _ => {}
        }

        if Some(elem.id) == model.editor.selected_id {
            let pulse = 0.7;
            match &elem.kind {
                crate::state::ElementKind::Bumper { radius, .. } => {
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
            if let crate::state::ElementKind::Flipper { side, length } = &elem.kind {
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

            // Renderation gizmo: axis line + draggable handle + live angle readout.
            let handle = crate::util::rotation_handle_pos(elem);
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
                labels,
                &model.settings,
                handle + Vec2::new(0.0, 0.35),
                format!("{:.0}°", deg),
                13.0,
                [1.0, 1.0, 0.2],
                0.95,
            );
        }
    }
}

pub(crate) fn draw_placement_preview(model: &Model, draw: &nannou::Draw, labels: &mut Vec<Label>) {
    if model.editor.tool == EditTool::Select || model.editor.tool == EditTool::Delete {
        return;
    }

    let pos = snap_to_grid(&model.settings, model.mouse_world);

    let kind = match model.editor.tool {
        EditTool::Wall => crate::state::ElementKind::Wall {
            width: model.editor.wall_width,
            height: model.editor.wall_height,
        },
        EditTool::Bumper => crate::state::ElementKind::Bumper {
            radius: model.editor.bumper_radius,
            boost: model.editor.bumper_boost,
            score: model.editor.bumper_score,
        },
        EditTool::FlipperLeft => crate::state::ElementKind::Flipper {
            side: FlipperSide::Left,
            length: crate::state::FLIPPER_LENGTH,
        },
        EditTool::FlipperRight => crate::state::ElementKind::Flipper {
            side: FlipperSide::Right,
            length: crate::state::FLIPPER_LENGTH,
        },
        EditTool::Chain => crate::state::ElementKind::Chain {
            link_count: model.editor.chain_links,
            total_length: model.editor.chain_length,
            end_mass: model.editor.chain_end_mass,
        },
        EditTool::FluidPool => crate::state::ElementKind::FluidPool {
            radius: model.editor.fluid_radius,
            fluid: model.editor.fluid_type,
            viscosity: model.editor.fluid_viscosity,
        },
        EditTool::SoftBridge => crate::state::ElementKind::SoftBridge {
            end_x: pos.x + 4.0,
            segments: model.editor.bridge_segments,
            softness: model.editor.bridge_softness,
        },
        EditTool::Target => crate::state::ElementKind::Target {
            width: model.editor.target_width,
            height: model.editor.target_height,
            score: model.editor.target_score,
        },
        EditTool::Drain => crate::state::ElementKind::Drain {
            width: model.editor.drain_width,
        },
        EditTool::BallSpawn => crate::state::ElementKind::BallSpawn,
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

    let extent = crate::util::element_extent(&ghost);
    draw.rect()
        .x_y(pos.x, pos.y)
        .w_h((extent + 0.5) * 2.0, (extent + 0.5) * 2.0)
        .no_fill()
        .stroke(rgba(0.6, 0.9, 1.0, 0.4))
        .stroke_weight(0.02);
    draw_world_label(
        labels,
        &model.settings,
        pos + Vec2::new(0.0, extent + 0.7),
        format!("({:.2}, {:.2})", pos.x, pos.y),
        13.0,
        [0.7, 0.95, 1.0],
        0.95,
    );
}
