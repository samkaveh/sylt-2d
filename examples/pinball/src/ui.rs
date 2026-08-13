use crate::state::{EditTool, FluidType, Model};
use nannou_egui::{self, egui};

pub(crate) struct EditorPanelActions {
    pub(crate) switch_to_play: bool,
    pub(crate) delete_selected: Option<usize>,
    pub(crate) do_save: bool,
    pub(crate) do_load: bool,
    pub(crate) do_clear: bool,
}

impl Default for EditorPanelActions {
    fn default() -> Self {
        EditorPanelActions {
            switch_to_play: false,
            delete_selected: None,
            do_save: false,
            do_load: false,
            do_clear: false,
        }
    }
}

pub(crate) struct PlayPanelActions {
    pub(crate) switch_to_edit: bool,
    pub(crate) do_restart: bool,
}

impl Default for PlayPanelActions {
    fn default() -> Self {
        PlayPanelActions {
            switch_to_edit: false,
            do_restart: false,
        }
    }
}

pub(crate) fn draw_editor_panel(model: &mut Model) -> EditorPanelActions {
    let mut actions = EditorPanelActions::default();
    {
        let editor = &mut model.editor;
        let settings = &mut model.settings;
        let board_name = &mut model.board_name_input;
        let ctx = model.egui.begin_frame();

        // Custom arcade dark theme styling for egui
        let mut style: egui::Style = (*ctx.style()).clone();
        style.visuals.dark_mode = true;
        style.visuals.widgets.noninteractive.bg_fill = egui::Color32::from_rgb(14, 16, 26);
        style.visuals.widgets.inactive.bg_fill = egui::Color32::from_rgb(22, 26, 42);
        style.visuals.widgets.hovered.bg_fill = egui::Color32::from_rgb(38, 48, 76);
        style.visuals.widgets.active.bg_fill = egui::Color32::from_rgb(0, 160, 220);
        style.visuals.window_fill = egui::Color32::from_rgb(14, 16, 26);
        style.visuals.selection.bg_fill = egui::Color32::from_rgb(0, 180, 240);
        ctx.set_style(style);

        egui::SidePanel::left("tools")
            .default_width(240.0)
            .show(&ctx, |ui| {
                ui.add_space(4.0);
                ui.heading(
                    egui::RichText::new("⚡ BOARD EDITOR")
                        .size(18.0)
                        .strong()
                        .color(egui::Color32::from_rgb(0, 210, 255)),
                );
                ui.add_space(2.0);
                ui.label(
                    egui::RichText::new("SYLT-2D Physics Playground")
                        .size(11.0)
                        .italics()
                        .color(egui::Color32::GRAY),
                );
                ui.separator();

                ui.label(
                    egui::RichText::new("TOOLBOX")
                        .size(12.0)
                        .strong()
                        .color(egui::Color32::LIGHT_BLUE),
                );

                // Categorized tool layout
                ui.collapsing("📦 Field Structures", |ui| {
                    ui.horizontal_wrapped(|ui| {
                        for (t, icon) in [
                            (EditTool::Select, "🎯 Select"),
                            (EditTool::Wall, "🧱 Wall"),
                            (EditTool::Bumper, "💥 Bumper"),
                            (EditTool::Target, "🎯 Target"),
                            (EditTool::Drain, "🔻 Drain"),
                        ] {
                            let selected = editor.tool == t;
                            if ui.selectable_label(selected, icon).clicked() {
                                editor.tool = t;
                            }
                        }
                    });
                });

                ui.collapsing("🕹 Controls & Flippers", |ui| {
                    ui.horizontal_wrapped(|ui| {
                        for (t, icon) in [
                            (EditTool::FlipperLeft, "◀ Left Flip"),
                            (EditTool::FlipperRight, "▶ Right Flip"),
                            (EditTool::BallSpawn, "📍 Spawn"),
                        ] {
                            let selected = editor.tool == t;
                            if ui.selectable_label(selected, icon).clicked() {
                                editor.tool = t;
                            }
                        }
                    });
                });

                ui.collapsing("💧 Deformables & Liquids", |ui| {
                    ui.horizontal_wrapped(|ui| {
                        for (t, icon) in [
                            (EditTool::FluidPool, "🌊 Fluid Pool"),
                            (EditTool::Chain, "🔗 Chain"),
                            (EditTool::SoftBridge, "🌉 Soft Bridge"),
                        ] {
                            let selected = editor.tool == t;
                            if ui.selectable_label(selected, icon).clicked() {
                                editor.tool = t;
                            }
                        }
                    });
                });

                if ui
                    .selectable_label(editor.tool == EditTool::Delete, "🗑 Delete Tool")
                    .clicked()
                {
                    editor.tool = EditTool::Delete;
                }

                ui.separator();
                ui.label(
                    egui::RichText::new("PROPERTIES")
                        .size(12.0)
                        .strong()
                        .color(egui::Color32::GOLD),
                );

                match editor.tool {
                    EditTool::Wall => {
                        ui.add(egui::Slider::new(&mut editor.wall_width, 0.5..=10.0).text("Width"));
                        ui.add(
                            egui::Slider::new(&mut editor.wall_height, 0.2..=5.0).text("Height"),
                        );
                    }
                    EditTool::Bumper => {
                        ui.add(
                            egui::Slider::new(&mut editor.bumper_radius, 0.3..=2.0).text("Radius"),
                        );
                        ui.add(
                            egui::Slider::new(&mut editor.bumper_boost, 3.0..=35.0)
                                .text("Boost Impulse"),
                        );
                        ui.add(
                            egui::Slider::new(&mut editor.bumper_score, 50..=1000)
                                .text("Score Value"),
                        );
                    }
                    EditTool::Chain => {
                        ui.add(
                            egui::Slider::new(&mut editor.chain_links, 2..=20).text("Link Count"),
                        );
                        ui.add(
                            egui::Slider::new(&mut editor.chain_length, 1.0..=10.0)
                                .text("Chain Length"),
                        );
                        ui.add(
                            egui::Slider::new(&mut editor.chain_end_mass, 5.0..=50.0)
                                .text("End Mass"),
                        );
                    }
                    EditTool::FluidPool => {
                        ui.add(
                            egui::Slider::new(&mut editor.fluid_radius, 0.5..=5.0)
                                .text("Pool Radius"),
                        );
                        ui.add(
                            egui::Slider::new(&mut editor.fluid_viscosity, 0.1..=3.0)
                                .text("Viscosity"),
                        );
                        ui.label("Fluid Type:");
                        ui.horizontal(|ui| {
                            for (ft, name) in [
                                (FluidType::Water, "💧 Water"),
                                (FluidType::Slime, "🟢 Slime"),
                                (FluidType::Lava, "🔥 Lava"),
                                (FluidType::Acid, "🧪 Acid"),
                            ] {
                                if ui.selectable_label(editor.fluid_type == ft, name).clicked() {
                                    editor.fluid_type = ft;
                                }
                            }
                        });
                    }
                    EditTool::SoftBridge => {
                        ui.add(
                            egui::Slider::new(&mut editor.bridge_segments, 3..=20).text("Segments"),
                        );
                        ui.add(
                            egui::Slider::new(&mut editor.bridge_softness, 0.005..=0.1)
                                .text("Softness"),
                        );
                    }
                    EditTool::Target => {
                        ui.add(
                            egui::Slider::new(&mut editor.target_width, 0.5..=3.0).text("Width"),
                        );
                        ui.add(
                            egui::Slider::new(&mut editor.target_height, 0.1..=1.0).text("Height"),
                        );
                        ui.add(
                            egui::Slider::new(&mut editor.target_score, 50..=2000).text("Score"),
                        );
                    }
                    EditTool::Drain => {
                        ui.add(
                            egui::Slider::new(&mut editor.drain_width, 1.0..=10.0).text("Width"),
                        );
                    }
                    _ => {
                        ui.label(
                            egui::RichText::new("Click on playfield to place selected tool.")
                                .italics()
                                .size(11.0),
                        );
                    }
                }

                if let Some(sel_id) = editor.selected_id {
                    ui.separator();
                    ui.label(
                        egui::RichText::new(format!("SELECTED ELEMENT: #{}", sel_id))
                            .strong()
                            .color(egui::Color32::YELLOW),
                    );
                    if let Some(elem) = model.elements.iter_mut().find(|e| e.id == sel_id) {
                        ui.add(
                            egui::Slider::new(
                                &mut elem.rotation,
                                -std::f32::consts::PI..=std::f32::consts::PI,
                            )
                            .text("Angle (rad)"),
                        );
                    }
                    if ui.button("🗑 Delete Selected").clicked() {
                        actions.delete_selected = Some(sel_id);
                    }
                }

                ui.separator();
                ui.label(
                    egui::RichText::new("BOARD FILE MANAGEMENT")
                        .size(12.0)
                        .strong()
                        .color(egui::Color32::LIGHT_GREEN),
                );
                ui.horizontal(|ui| {
                    ui.label("Name:");
                    ui.text_edit_singleline(board_name);
                });
                ui.horizontal(|ui| {
                    if ui.button("💾 Save").clicked() {
                        actions.do_save = true;
                    }
                    if ui.button("📂 Load").clicked() {
                        actions.do_load = true;
                    }
                    if ui.button("❌ Clear").clicked() {
                        actions.do_clear = true;
                    }
                });

                ui.separator();
                ui.label(
                    egui::RichText::new("VIEW & GRID")
                        .size(12.0)
                        .strong()
                        .color(egui::Color32::LIGHT_BLUE),
                );
                ui.checkbox(&mut settings.show_grid, "Show Grid");
                ui.checkbox(&mut settings.snap_to_grid, "Snap to Grid");
                if settings.snap_to_grid {
                    ui.add(
                        egui::Slider::new(&mut settings.grid_size, 0.1..=1.0).text("Grid Spacing"),
                    );
                }
                ui.checkbox(&mut settings.show_contacts, "Show Contact Points");
                ui.add(egui::Slider::new(&mut settings.scale, 10.0..=50.0).text("Zoom Scale"));

                ui.separator();
                ui.add_space(4.0);
                if ui
                    .button(
                        egui::RichText::new("▶ PLAY TEST  [TAB]")
                            .size(14.0)
                            .strong()
                            .color(egui::Color32::GREEN),
                    )
                    .clicked()
                {
                    actions.switch_to_play = true;
                }
            });
    }
    actions
}

pub(crate) fn draw_play_panel(model: &mut Model) -> PlayPanelActions {
    let mut actions = PlayPanelActions::default();
    {
        let settings = &mut model.settings;
        let play = &model.play;
        let ctx = model.egui.begin_frame();

        // Custom dark theme styling
        let mut style: egui::Style = (*ctx.style()).clone();
        style.visuals.dark_mode = true;
        style.visuals.window_fill = egui::Color32::from_rgb(14, 16, 26);
        ctx.set_style(style);

        egui::SidePanel::right("play_hud")
            .default_width(175.0)
            .show(&ctx, |ui| {
                ui.add_space(4.0);
                ui.heading(
                    egui::RichText::new("🎰 PINBALL HUD")
                        .size(16.0)
                        .strong()
                        .color(egui::Color32::GOLD),
                );
                ui.separator();

                egui::Frame::default()
                    .fill(egui::Color32::from_rgb(20, 24, 38))
                    .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(0, 180, 240)))
                    .rounding(6.0)
                    .inner_margin(8.0)
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new("SCORE")
                                .size(11.0)
                                .color(egui::Color32::LIGHT_BLUE),
                        );
                        ui.label(
                            egui::RichText::new(format!("{}", play.score))
                                .size(24.0)
                                .strong()
                                .color(egui::Color32::YELLOW),
                        );
                    });

                if play.combo_multiplier > 1 {
                    ui.add_space(2.0);
                    ui.label(
                        egui::RichText::new(format!("🔥 COMBO x{}", play.combo_multiplier))
                            .size(14.0)
                            .color(egui::Color32::from_rgb(255, 130, 0))
                            .strong(),
                    );
                }

                ui.add_space(2.0);
                ui.label(
                    egui::RichText::new(format!("HIGH SCORE: {}", play.high_score))
                        .size(11.0)
                        .color(egui::Color32::GRAY),
                );
                ui.separator();

                ui.label(
                    egui::RichText::new("BALLS REMAINING")
                        .size(11.0)
                        .strong()
                        .color(egui::Color32::LIGHT_BLUE),
                );
                ui.horizontal(|ui| {
                    for i in 0..3 {
                        let filled = (i as u32) < play.balls_remaining;
                        let icon = if filled { "⚪" } else { "⚫" };
                        ui.label(egui::RichText::new(icon).size(16.0));
                    }
                });

                if play.ball_save_timer > 0.0 {
                    ui.label(
                        egui::RichText::new("🛡 BALL SAVE ACTIVE")
                            .size(11.0)
                            .strong()
                            .color(egui::Color32::GREEN),
                    );
                }

                ui.separator();
                egui::CollapsingHeader::new("🕹 Controls")
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.label("Left Flipper:  A / ←");
                        ui.label("Right Flipper: D / →");
                        ui.label("Plunger:       SPACE");
                        ui.label("Reset:         R");
                    });

                if play.game_over {
                    ui.separator();
                    ui.colored_label(egui::Color32::RED, "💥 GAME OVER 💥");
                    if ui
                        .button(
                            egui::RichText::new("🔄 Restart")
                                .strong()
                                .color(egui::Color32::YELLOW),
                        )
                        .clicked()
                    {
                        actions.do_restart = true;
                    }
                }

                ui.separator();
                ui.label(
                    egui::RichText::new("VIEW OPTIONS")
                        .size(11.0)
                        .strong()
                        .color(egui::Color32::LIGHT_BLUE),
                );
                ui.add(egui::Slider::new(&mut settings.scale, 10.0..=50.0).text("Zoom"));
                ui.checkbox(&mut settings.show_contacts, "Show Contacts");
                ui.separator();
                ui.add_space(4.0);
                if ui
                    .button(
                        egui::RichText::new("✏ EDIT BOARD [TAB]")
                            .strong()
                            .color(egui::Color32::LIGHT_BLUE),
                    )
                    .clicked()
                {
                    actions.switch_to_edit = true;
                }
            });
    }
    actions
}
