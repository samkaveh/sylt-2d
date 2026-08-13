use crate::board::{enter_editor_mode, enter_play_mode};
use crate::physics::launch_plunger;
use crate::state::{
    BoardElement, DemoPhase, EditTool, ElementKind, FlipperSide, FluidType, GameMode, Model,
    FLIPPER_LENGTH,
};
use crate::util::default_ball_spawn;
use sylt_2d::math_utils::Vec2;

/// The playfield used by the demo tour. Exercises every physics feature at
/// once: flippers, bumpers, drop targets, an all fluid types, a soft bridge
/// and a chain that the ball bounces into.
pub(crate) fn demo_build_play_board(model: &mut Model) {
    fn push(model: &mut Model, position: Vec2, rotation: f32, kind: ElementKind, color: [f32; 3]) {
        model.elements.push(BoardElement {
            id: model.next_id,
            position,
            rotation,
            kind,
            color,
        });
        model.next_id += 1;
    }

    // Bumpers in a triangle up top (they flash + boost the ball).
    push(
        model,
        Vec2::new(-2.0, 14.0),
        0.0,
        ElementKind::Bumper {
            radius: 0.8,
            boost: 22.0,
            score: 100,
        },
        [0.95, 0.35, 0.2],
    );
    push(
        model,
        Vec2::new(2.0, 14.0),
        0.0,
        ElementKind::Bumper {
            radius: 0.8,
            boost: 22.0,
            score: 100,
        },
        [0.95, 0.35, 0.2],
    );
    push(
        model,
        Vec2::new(0.0, 16.5),
        0.0,
        ElementKind::Bumper {
            radius: 0.9,
            boost: 26.0,
            score: 250,
        },
        [1.0, 0.8, 0.1],
    );

    // Drop targets on the sides — repositioned to be reachable.
    push(
        model,
        Vec2::new(-4.0, 11.0),
        0.3,
        ElementKind::Target {
            width: 1.2,
            height: 0.35,
            score: 500,
        },
        [0.2, 0.8, 0.9],
    );
    push(
        model,
        Vec2::new(3.0, 11.0),
        -0.3,
        ElementKind::Target {
            width: 1.2,
            height: 0.35,
            score: 500,
        },
        [0.2, 0.8, 0.9],
    );
    // Extra target near the center for more scoring opportunities
    push(
        model,
        Vec2::new(0.0, 8.0),
        0.0,
        ElementKind::Target {
            width: 1.5,
            height: 0.35,
            score: 750,
        },
        [0.9, 0.6, 0.9],
    );

    // Flippers (left/right) at the bottom.
    push(
        model,
        Vec2::new(-3.2, -0.6),
        -0.4,
        ElementKind::Flipper {
            side: FlipperSide::Left,
            length: FLIPPER_LENGTH,
        },
        [0.1, 0.75, 0.5],
    );
    push(
        model,
        Vec2::new(1.6, -0.6),
        0.4,
        ElementKind::Flipper {
            side: FlipperSide::Right,
            length: FLIPPER_LENGTH,
        },
        [0.1, 0.75, 0.5],
    );

    // A soft bridge spans a mid-descent path: the ball bounces/settles on it.
    push(
        model,
        Vec2::new(-4.0, 7.0),
        0.0,
        ElementKind::SoftBridge {
            end_x: 4.0,
            segments: 10,
            softness: 0.03,
        },
        [0.75, 0.6, 0.3],
    );

    // A hanging chain acts as a pendulum that the ball can swing through.
    push(
        model,
        Vec2::new(-2.0, 10.0),
        0.0,
        ElementKind::Chain {
            link_count: 5,
            total_length: 2.5,
            end_mass: 12.0,
        },
        [0.6, 0.6, 0.8],
    );

    // Small guide walls on the sides — positioned high and angled to funnel
    // the ball through the bumper zone without blocking the main path.
    push(
        model,
        Vec2::new(-5.5, 12.0),
        -0.5,
        ElementKind::Wall {
            width: 0.8,
            height: 2.5,
        },
        [0.4, 0.45, 0.6],
    );
    push(
        model,
        Vec2::new(4.0, 12.0),
        0.5,
        ElementKind::Wall {
            width: 0.8,
            height: 2.5,
        },
        [0.4, 0.45, 0.6],
    );

    // Fluid pools spread across the playfield so the ball encounters each one.
    let pools = [
        (Vec2::new(-2.2, 5.5), 1.6, FluidType::Water),
        (Vec2::new(2.2, 7.5), 1.5, FluidType::Slime),
        (Vec2::new(-3.5, 9.5), 1.4, FluidType::Acid),
        (Vec2::new(0.0, 3.2), 1.5, FluidType::Lava),
    ];
    for (pos, radius, fluid) in pools {
        push(
            model,
            pos,
            0.0,
            ElementKind::FluidPool {
                radius,
                fluid,
                viscosity: 0.8,
            },
            [0.5, 0.5, 0.5],
        );
    }

    // Drain at the bottom and the plunger-lane spawn point.
    push(
        model,
        Vec2::new(0.0, -5.0),
        0.0,
        ElementKind::Drain { width: 5.0 },
        [0.85, 0.2, 0.2],
    );
    push(
        model,
        default_ball_spawn(),
        0.0,
        ElementKind::BallSpawn,
        [0.9, 0.9, 0.9],
    );
}

/// Ordered tour of the editor tools shown in the second half of the demo.
const EDITOR_TOOL_TOUR: [EditTool; 11] = [
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
];

/// Builds the on-screen element for the given editor tool, sized/placed so it
/// is clearly visible on the blank editor board.
pub(crate) fn demo_edit_tool_element(model: &Model, tool: EditTool) -> BoardElement {
    let pos = Vec2::new(0.0, 6.0);
    let kind = match tool {
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
            link_count: 7,
            total_length: 3.5,
            end_mass: 16.0,
        },
        EditTool::FluidPool => ElementKind::FluidPool {
            radius: 1.5,
            fluid: FluidType::Water,
            viscosity: 0.8,
        },
        EditTool::SoftBridge => ElementKind::SoftBridge {
            end_x: pos.x + 5.0,
            segments: 10,
            softness: 0.03,
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
        _ => ElementKind::Wall {
            width: 1.0,
            height: 1.0,
        },
    };
    let rotation = match tool {
        EditTool::FlipperLeft => -0.45,
        EditTool::FlipperRight => 0.45,
        _ => 0.0,
    };
    BoardElement {
        id: usize::MAX,
        position: pos,
        rotation,
        kind,
        color: [0.5, 0.9, 1.0],
    }
}

/// Rebuilds the editor board to showcase the current tool of the tour.
pub(crate) fn demo_editor_show_tool(model: &mut Model) {
    let idx = model.demo.editor_tool_idx;
    let tool = EDITOR_TOOL_TOUR[idx];
    model.editor.tool = tool;

    // Show the tool name prominently on the caption.
    model.demo.caption = format!("Editor tool - {}", tool.name());

    // Select / Delete have nothing to place.
    if matches!(tool, EditTool::Select | EditTool::Delete) {
        model.elements.clear();
        return;
    }
    let mut elem = demo_edit_tool_element(model, tool);
    elem.id = model.next_id;
    model.next_id += 1;
    model.elements.clear();
    model.elements.push(elem);
}

pub(crate) fn update_demo(model: &mut Model) {
    let dt = model.time_step;
    model.demo.timer += dt;

    match model.demo.phase {
        DemoPhase::Init => {
            model.demo.timer = 0.0;
            model.demo.caption = "SYLT-2D PINBALL DEMO".to_string();
            model.demo.phase = DemoPhase::BuildBoard;
        }
        DemoPhase::BuildBoard => {
            if model.demo.timer > 0.4 {
                model.elements.clear();
                demo_build_play_board(model);
                model.demo.timer = 0.0;
                model.demo.caption =
                    "Building playfield: Water, Slime, Acid & Lava fluid pools + physics targets"
                        .to_string();
                model.demo.phase = DemoPhase::SwitchToPlay;
            }
        }
        DemoPhase::SwitchToPlay => {
            if model.demo.timer > 2.2 {
                model.mode = GameMode::Play;
                enter_play_mode(model);
                model.demo.timer = 0.0;
                model.demo.caption =
                    "Playing - watch ball interactions with SPH fluid pools".to_string();
                model.demo.phase = DemoPhase::WaitAfterPlay;
            }
        }
        DemoPhase::WaitAfterPlay => {
            if model.demo.timer > 1.5 {
                model.demo.timer = 0.0;
                model.demo.caption = "Plunger: hold to charge, release to launch".to_string();
                model.demo.phase = DemoPhase::ChargePlunger;
            }
        }
        DemoPhase::ChargePlunger => {
            model.play.plunger_charging = true;
            if model.demo.timer > 2.0 {
                model.play.plunger_charging = false;
                model.demo.timer = 0.0;
                model.demo.caption = "Launch!".to_string();
                model.demo.phase = DemoPhase::LaunchBall;
            }
        }
        DemoPhase::LaunchBall => {
            launch_plunger(model);
            model.demo.timer = 0.0;
            model.demo.caption =
                "Flippers keep the ball in motion through the fluid pools".to_string();
            model.demo.phase = DemoPhase::BallInPlay;
        }
        DemoPhase::BallInPlay => {
            // Smart AI: track the ball and activate the appropriate flipper.
            let mut active_fluid: Option<FluidType> = None;

            if let Some(ball_id) = model.play.ball_body_id {
                if let Some(ball_ref) = model.world.bodies.iter().find(|b| b.borrow().id == ball_id)
                {
                    let ball = ball_ref.borrow();
                    let bpos = ball.position;
                    let vy = ball.velocity.y;

                    // Check if ball is inside any fluid pool
                    for elem in &model.elements {
                        if let ElementKind::FluidPool { radius, fluid, .. } = elem.kind {
                            if (bpos - elem.position).length() < radius {
                                active_fluid = Some(fluid);
                                break;
                            }
                        }
                    }

                    // Fire flippers to maintain ball in play
                    let in_range = bpos.y < 6.0 && bpos.y > -2.0;
                    let falling = vy < -1.0;
                    model.play.flipper_left_active = in_range && (falling || bpos.y < 2.0);
                    model.play.flipper_right_active = in_range && (falling || bpos.y < 2.0);
                }
            } else {
                model.play.flipper_left_active = false;
                model.play.flipper_right_active = false;
            }

            // Update dynamic fluid interaction captions
            if let Some(fluid) = active_fluid {
                model.demo.caption = match fluid {
                    FluidType::Water => {
                        "💧 Water Pool: Hydrodynamic waves, splash bursts & smooth drag".to_string()
                    }
                    FluidType::Slime => {
                        "🟢 Slime Pool: High viscosity dampening & thick fluid resistance"
                            .to_string()
                    }
                    FluidType::Lava => {
                        "🔥 Lava Pool: High energy heat glow & molten core bursts".to_string()
                    }
                    FluidType::Acid => {
                        "🧪 Acid Pool: Corrosive bubbling & toxic particle splash".to_string()
                    }
                };
            } else if model.demo.timer > 4.0 && model.demo.timer < 10.0 {
                model.demo.caption =
                    "Flippers keep the ball in motion through the fluid pools".to_string();
            }

            if model.demo.timer > 14.0 || model.play.game_over {
                model.play.flipper_left_active = false;
                model.play.flipper_right_active = false;
                model.demo.timer = 0.0;
                model.demo.caption = "Now the editor - every tool in turn".to_string();
                model.demo.phase = DemoPhase::WatchBall;
            }
        }
        DemoPhase::WatchBall => {
            if model.demo.timer > 1.5 {
                model.demo.timer = 0.0;
                model.mode = GameMode::Editor;
                enter_editor_mode(model);
                model.demo.editor_tool_idx = 0;
                demo_editor_show_tool(model);
                model.demo.phase = DemoPhase::EditorTools;
            }
        }
        DemoPhase::EditorTools => {
            // Show the very first tool, then advance through the tour.
            if model.demo.timer > 2.2 {
                model.demo.editor_tool_idx += 1;
                if model.demo.editor_tool_idx >= EDITOR_TOOL_TOUR.len() {
                    model.demo.timer = 0.0;
                    model.demo.phase = DemoPhase::Done;
                } else {
                    demo_editor_show_tool(model);
                    model.demo.timer = 0.0;
                }
            }
        }
        DemoPhase::Done => {
            model.demo.caption = "Demo complete - restarting...".to_string();
            if model.demo.timer > 3.0 {
                model.demo.timer = 0.0;
                model.mode = GameMode::Editor;
                enter_editor_mode(model);
                model.elements.clear();
                model.demo.editor_tool_idx = 0;
                model.demo.phase = DemoPhase::BuildBoard;
            }
        }
    }
}
