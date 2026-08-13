use crate::state::{
    BoardElement, ElementKind, FlipperSide, FluidType, Model, BALL_RADIUS, BALL_SAVE_DURATION,
    FLIPPER_LENGTH, FLIPPER_UP_DELTA, FLIPPER_WIDTH,
};
use crate::util::default_ball_spawn;
use sylt_2d::body::Body;
use sylt_2d::joint::Joint;
use sylt_2d::math_utils::{Mat2x2, Vec2};

pub(crate) fn spawn_ball(model: &mut Model) {
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

pub(crate) fn add_plunger(model: &mut Model) {
    // Plunger resting at the bottom of plunger lane (x: 6.6..7.8, y: -3.2)
    let mut plunger = Body::new(Vec2::new(1.1, 0.8), f32::MAX);
    plunger.position = Vec2::new(7.2, -3.2);
    plunger.friction = 0.05;
    model.play.plunger_body_id = Some(plunger.id);
    model.world.add_body(plunger);
}

pub(crate) fn spawn_fluid_pool(model: &mut Model, center: Vec2, radius: f32, fluid: FluidType) {
    // A fluid pool consists of dynamic particle bodies that jostle under SPH fluid
    // dynamics (density pressure, surface tension, and drag). They are free-floating
    // so they slosh, wave, and flow naturally around the pinball.
    let particle_r = 0.22;
    let spacing = 0.38;
    let color = match fluid {
        FluidType::Water => [0.2, 0.55, 0.95],
        FluidType::Slime => [0.3, 0.85, 0.25],
        FluidType::Lava => [0.95, 0.4, 0.12],
        FluidType::Acid => [0.7, 0.9, 0.1],
    };
    let particle_mass = match fluid {
        FluidType::Water => 0.05,
        FluidType::Slime => 0.08,
        FluidType::Lava => 0.10,
        FluidType::Acid => 0.06,
    };

    let mut x = -radius + particle_r;
    let mut row = 0i32;
    while x <= radius - particle_r {
        let y_offset = if row % 2 == 0 { 0.0 } else { spacing * 0.5 };
        let mut y = -radius + particle_r;
        while y <= radius - particle_r {
            let p = Vec2::new(x, y + y_offset);
            if p.length() <= radius - particle_r {
                let mut body = Body::new_circle(particle_r, particle_mass);
                body.position = center + p;
                body.friction = 0.02;
                // Sensors collide with sensors to jostle each other into fluid volumes,
                // while pinball momentum is transferred via hydrodynamic SPH forces.
                body.set_sensor(true);
                model.play.metaball_bodies.push(body.id);
                model.play.metaball_colors.insert(body.id, color);
                model.play.metaball_fluid_types.insert(body.id, fluid);
                model.play.fluid_particle_ids.push(body.id);
                model.world.add_body(body.clone());
            }
            y += spacing;
        }
        x += spacing * 0.866;
        row += 1;
    }
}

pub(crate) fn add_cabinet_walls(model: &mut Model) {
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

pub(crate) fn populate_default_board(model: &mut Model) {
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

    // Fluid Pool in mid-field
    model.elements.push(BoardElement {
        id: model.next_id,
        position: Vec2::new(0.0, 7.5),
        rotation: 0.0,
        kind: ElementKind::FluidPool {
            radius: 2.2,
            fluid: FluidType::Water,
            viscosity: 0.8,
        },
        color: [0.2, 0.55, 0.95],
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

pub(crate) fn build_element_bodies(model: &mut Model) {
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
                let rot_mat = Mat2x2::new_from_angle(rot_init);
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
                fluid,
                viscosity: _,
            } => {
                spawn_fluid_pool(model, elem.position, *radius, *fluid);
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

pub(crate) fn enter_play_mode(model: &mut Model) {
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
    model.play.metaball_colors.clear();
    model.play.metaball_fluid_types.clear();
    model.play.fluid_particle_ids.clear();
    model.play.fluid_drag_active = false;
    model.play.bumper_flash_timers.clear();
    model.play.ball_trail.clear();
    model.play.bumper_cooldowns.clear();
    model.play.target_cooldowns.clear();
    model.play.combo_multiplier = 1;
    model.play.combo_timer = 0.0;
    model.play.ball_save_timer = BALL_SAVE_DURATION;
    model.play.score_popups.clear();
    model.play.bumper_particles.clear();

    if model.elements.is_empty() {
        populate_default_board(model);
    }

    add_cabinet_walls(model);
    add_plunger(model);
    build_element_bodies(model);
    spawn_ball(model);
}

pub(crate) fn enter_editor_mode(model: &mut Model) {
    model.world.clear();
    model.play.ball_body_id = None;
    model.play.flipper_left_id = None;
    model.play.flipper_right_id = None;
    model.play.plunger_body_id = None;
    model.play.chain_bodies.clear();
    model.play.chain_anchor_id = None;
    model.play.metaball_bodies.clear();
    model.play.metaball_colors.clear();
    model.play.metaball_fluid_types.clear();
    model.play.fluid_particle_ids.clear();
    if model.elements.is_empty() {
        populate_default_board(model);
    }
    // Keep the board frame (walls, arch, plunger) visible as a reference
    // while editing, so elements can be placed in context.
    add_cabinet_walls(model);
    add_plunger(model);
}
