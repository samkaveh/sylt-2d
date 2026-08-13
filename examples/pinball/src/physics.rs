use crate::board::spawn_ball;
use crate::state::{
    BALL_MAX_SPEED, BALL_RADIUS, BALL_SAVE_DURATION, BUMPER_COOLDOWN, COMBO_WINDOW, FLIPPER_LENGTH,
    ElementKind, FluidType, Model, PLUNGER_MAX_CHARGE, TARGET_COOLDOWN,
};
use crate::util::{default_ball_spawn, rotate_vec, vec2_normalize};
use sylt_2d::body::Body;
use sylt_2d::math_utils::Vec2;

pub(crate) fn apply_fluid_drag(model: &mut Model) {
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

/// Gives the liquid particles organic wobble so the metaball contour morphs
/// like a liquid surface, and gently parts them out of the ball's way so the
/// passing pinball carves a channel through the blob. The particles bump each
/// other (sensors collide with sensors) but never block the pinball itself.
pub(crate) fn animate_fluid_particles(model: &mut Model, dt: f32) {
    model.play.fluid_time += dt;
    let t = model.play.fluid_time;
    let amp = 0.35f32;

    let ball_pos = model.play.ball_body_id.and_then(|ball_id| {
        model
            .world
            .bodies
            .iter()
            .find(|b| b.borrow().id == ball_id)
            .map(|b| b.borrow().position)
    });
    let ball_radius = 0.35f32;

    for &id in &model.play.fluid_particle_ids {
        let opt_body = model.world.bodies.iter().find(|b| b.borrow().id == id);
        if let Some(body) = opt_body {
            let mut body = body.borrow_mut();

            // Slow organic ripples keyed to particle id so neighbouring
            // particles don't all move in sync (that would look rigid).
            let phase = id as f32 * 1.7;
            let ripple = Vec2::new(
                (t * 2.1 + phase).sin(),
                (t * 2.7 + phase * 1.3).cos(),
            ) * amp;

            // Part the blob around the pinball without ever pushing the ball.
            let mut displace = Vec2::new(0.0, 0.0);
            if let Some(bp) = ball_pos {
                let d = body.position - bp;
                let dist = d.length();
                let r_sum = ball_radius + body.radius + 0.15;
                if dist > f32::EPSILON && dist < r_sum {
                    let n = d * (1.0 / dist);
                    displace = n * (r_sum - dist) * 45.0;
                }
            }

            body.add_force(ripple + displace);
        }
    }
}

pub(crate) fn detect_scoring(model: &mut Model) {
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

        // Ball speed cap
        if let Some(body_ref) = model.world.bodies.iter().find(|b| b.borrow().id == ball_id) {
            let mut body = body_ref.borrow_mut();
            let speed = body.velocity.length();
            if speed > BALL_MAX_SPEED {
                body.velocity = body.velocity * (BALL_MAX_SPEED / speed);
            }
        }

        // Update cooldown timers
        model.play.bumper_cooldowns.retain_mut(|(_, timer)| {
            *timer -= model.time_step;
            *timer > 0.0
        });
        model.play.target_cooldowns.retain_mut(|(_, timer)| {
            *timer -= model.time_step;
            *timer > 0.0
        });

        // Update combo timer
        if model.play.combo_timer > 0.0 {
            model.play.combo_timer -= model.time_step;
            if model.play.combo_timer <= 0.0 {
                model.play.combo_multiplier = 1;
            }
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
                        // Check cooldown
                        let on_cooldown = model
                            .play
                            .bumper_cooldowns
                            .iter()
                            .any(|(pos, _)| (*pos - elem.position).length() < 0.1);
                        if on_cooldown {
                            continue;
                        }

                        let body_opt = model.world.bodies.iter().find(|b| b.borrow().id == ball_id);
                        if let Some(body_ref) = body_opt {
                            let mut body = body_ref.borrow_mut();
                            let dir = vec2_normalize(body.position - elem.position);
                            if body.velocity.dot(dir) < *boost {
                                body.velocity = dir * *boost;

                                // Apply combo multiplier
                                model.play.combo_timer = COMBO_WINDOW;
                                let points = *score * model.play.combo_multiplier;
                                model.play.score += points;
                                model.play.combo_multiplier =
                                    (model.play.combo_multiplier + 1).min(10);
                                model
                                    .play
                                    .bumper_flash_timers
                                    .push((elem.position, 0.2));
                                model
                                    .play
                                    .bumper_cooldowns
                                    .push((elem.position, BUMPER_COOLDOWN));
                                model
                                    .play
                                    .score_popups
                                    .push((elem.position, points, 0.8));
                                // Spawn a particle burst radiating outward from the bumper
                                for _ in 0..10 {
                                    let angle = std::f32::consts::TAU
                                        * (model.next_id as f32 * 0.6180339887).fract();
                                    let speed = 3.0 + (model.next_id % 4) as f32;
                                    model.play.bumper_particles.push((
                                        elem.position,
                                        Vec2::new(angle.cos() * speed, angle.sin() * speed),
                                        0.3,
                                    ));
                                    model.next_id += 1;
                                }
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
                        // Check cooldown per element id
                        let on_cooldown = model
                            .play
                            .target_cooldowns
                            .iter()
                            .any(|(id, _)| *id == elem.id);
                        if !on_cooldown {
                            model.play.combo_timer = COMBO_WINDOW;
                            let points = *score * model.play.combo_multiplier;
                            model.play.score += points;
                            model.play.combo_multiplier =
                                (model.play.combo_multiplier + 1).min(10);
                            model
                                .play
                                .target_cooldowns
                                .push((elem.id, TARGET_COOLDOWN));
                            model.play.score_popups.push((elem.position, points, 0.8));
                        }
                    }
                }
                _ => {}
            }
        }

        // Update score popups
        model.play.score_popups.retain_mut(|(_, _, timer)| {
            *timer -= model.time_step;
            *timer > 0.0
        });

        // Drain condition with ball save
        if ball_pos.y < -3.5 {
            if model.play.ball_save_timer > 0.0 {
                // Ball save: respawn at plunger lane
                if let Some(body_ref) =
                    model.world.bodies.iter().find(|b| b.borrow().id == ball_id)
                {
                    let mut body = body_ref.borrow_mut();
                    body.position = default_ball_spawn();
                    body.velocity = Vec2::new(0.0, 0.0);
                }
                model.play.ball_save_timer = 0.0;
            } else {
                model.play.balls_remaining = model.play.balls_remaining.saturating_sub(1);
                model.play.combo_multiplier = 1;
                model.play.combo_timer = 0.0;
                if model.play.balls_remaining == 0 {
                    model.play.game_over = true;
                    if model.play.score > model.play.high_score {
                        model.play.high_score = model.play.score;
                    }
                } else {
                    model.world.bodies.retain(|b| b.borrow().id != ball_id);
                    model.play.ball_body_id = None;
                    spawn_ball(model);
                    model.play.ball_save_timer = BALL_SAVE_DURATION;
                }
            }
        }
    }
}

pub(crate) fn apply_flippers(model: &mut Model) {
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
pub(crate) fn drive_flipper(body: &mut Body, target: f32, pivot_offset: Vec2) {
    const PROPORTIONAL_GAIN: f32 = 60.0;
    const MAX_SPEED: f32 = 42.0;
    const SETTLE_ANGLE: f32 = 0.03;
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

pub(crate) fn apply_plunger(model: &mut Model) {
    if model.play.plunger_charging {
        model.play.plunger_charge = (model.play.plunger_charge + 1.2).min(PLUNGER_MAX_CHARGE);
    }
}

pub(crate) fn launch_plunger(model: &mut Model) {
    if model.play.plunger_charge > 0.0 {
        if let Some(ball_id) = model.play.ball_body_id {
            if let Some(body_ref) = model.world.bodies.iter().find(|b| b.borrow().id == ball_id) {
                let mut body = body_ref.borrow_mut();
                // Only launch when the ball is on the launch station (plunger lane).
                let in_lane = body.position.x > 5.0 && body.position.y < 3.0;
                if in_lane {
                    body.velocity = Vec2::new(0.0, model.play.plunger_charge);
                }
            }
        }
        model.play.plunger_charge = 0.0;
        model.play.plunger_charging = false;
    }
}