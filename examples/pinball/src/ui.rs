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
                        actions.delete_selected = Some(sel_id);
                    }
                }
                ui.separator();
                ui.label("Board Management:");
                ui.horizontal(|ui| {
                    ui.label("Name:");
                    ui.text_edit_singleline(board_name);
                });
                if ui.button("Save Board").clicked() {
                    actions.do_save = true;
                }
                if ui.button("Load Board").clicked() {
                    actions.do_load = true;
                }
                if ui.button("Clear All").clicked() {
                    actions.do_clear = true;
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

        egui::SidePanel::right("play_hud")
            .default_width(160.0)
            .show(&ctx, |ui| {
                ui.heading("🎰 PINBALL");
                ui.separator();
                egui::Frame::default()
                    .fill(egui::Color32::from_rgb(20, 22, 30))
                    .inner_margin(6.0)
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new(format!("SCORE\n{}", play.score))
                                .size(22.0)
                                .strong()
                                .color(egui::Color32::YELLOW),
                        );
                    });
                if play.combo_multiplier > 1 {
                    ui.label(
                        egui::RichText::new(format!("COMBO x{}", play.combo_multiplier))
                            .color(egui::Color32::from_rgb(255, 140, 0))
                            .strong(),
                    );
                }
                ui.label(format!("HIGH SCORE: {}", play.high_score));
                ui.separator();
                // Ball counter as icons
                ui.label("BALLS:");
                ui.horizontal(|ui| {
                    for i in 0..3 {
                        let filled = (i as u32) < play.balls_remaining;
                        let icon = if filled { "●" } else { "○" };
                        ui.label(
                            egui::RichText::new(icon)
                                .size(18.0)
                                .color(if filled {
                                    egui::Color32::LIGHT_BLUE
                                } else {
                                    egui::Color32::DARK_GRAY
                                }),
                        );
                    }
                });
                if play.ball_save_timer > 0.0 {
                    ui.label(
                        egui::RichText::new("BALL SAVE ACTIVE")
                            .size(12.0)
                            .color(egui::Color32::LIGHT_GREEN),
                    );
                }
                ui.separator();
                egui::CollapsingHeader::new("Controls")
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.label("Left Flipper:  A / ←");
                        ui.label("Right Flipper: D / →");
                        ui.label("Plunger:       SPACE");
                        ui.label("Reset:         R");
                    });
                ui.separator();
                if play.game_over {
                    ui.colored_label(egui::Color32::RED, "💥 GAME OVER 💥");
                    if ui.button("🔄 Restart").clicked() {
                        actions.do_restart = true;
                    }
                }
                ui.separator();
                ui.label("View:");
                ui.add(egui::Slider::new(&mut settings.scale, 10.0..=50.0).text("Zoom"));
                ui.checkbox(&mut settings.show_contacts, "Show Contacts");
                ui.separator();
                if ui.button("✏ Edit  [TAB]").clicked() {
                    actions.switch_to_edit = true;
                }
            });
    }
    actions
}
