use crate::state::{
    BoardElement, EguiSettings, ElementKind, FlipperSide, BALL_RADIUS, FLIPPER_WIDTH,
};
use nannou::prelude::*;
use sylt_2d::math_utils::Vec2;

/// Default Spawn Position inside the Plunger Lane
pub(crate) fn default_ball_spawn() -> Vec2 {
    Vec2::new(7.0, -1.0)
}

pub(crate) fn vec2_normalize(v: Vec2) -> Vec2 {
    let len = v.length();
    if len < f32::EPSILON {
        Vec2::new(0.0, 0.0)
    } else {
        v * (1.0 / len)
    }
}

pub(crate) fn rotate_vec(v: Vec2, angle: f32) -> Vec2 {
    Vec2::new(
        v.x * angle.cos() - v.y * angle.sin(),
        v.x * angle.sin() + v.y * angle.cos(),
    )
}

pub(crate) fn snap_value(v: f32, step: f32) -> f32 {
    if step <= 0.0 {
        v
    } else {
        (v / step).round() * step
    }
}

pub(crate) fn dist_point_segment(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let len_sq = ab.dot(ab);
    if len_sq < f32::EPSILON {
        return (p - a).length();
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    (p - (a + ab * t)).length()
}

pub(crate) fn element_hit(elem: &BoardElement, pos: Vec2) -> bool {
    match &elem.kind {
        ElementKind::Wall { width, height } | ElementKind::Target { width, height, .. } => {
            let local = rotate_vec(pos - elem.position, -elem.rotation);
            local.x.abs() < width * 0.5 + 0.3 && local.y.abs() < height * 0.5 + 0.3
        }
        ElementKind::Bumper { radius, .. } | ElementKind::FluidPool { radius, .. } => {
            (pos - elem.position).length() < *radius + 0.3
        }
        ElementKind::Flipper { side, length } => {
            let offset_x = match side {
                FlipperSide::Left => length * 0.45,
                FlipperSide::Right => -length * 0.45,
            };
            let tip = elem.position + rotate_vec(Vec2::new(offset_x, 0.0), elem.rotation);
            dist_point_segment(pos, elem.position, tip) < FLIPPER_WIDTH * 0.5 + 0.3
        }
        ElementKind::Drain { width } => {
            let local = rotate_vec(pos - elem.position, -elem.rotation);
            local.x.abs() < width * 0.5 + 0.3 && local.y.abs() < 0.45
        }
        _ => {
            let dx = (pos.x - elem.position.x).abs();
            let dy = (pos.y - elem.position.y).abs();
            dx < 1.5 && dy < 1.5
        }
    }
}

pub(crate) fn element_extent(elem: &BoardElement) -> f32 {
    match &elem.kind {
        ElementKind::Wall { width, height } => width.max(*height) * 0.5,
        ElementKind::Bumper { radius, .. } => *radius,
        ElementKind::Flipper { length, .. } => length * 0.5,
        ElementKind::Chain { total_length, .. } => total_length * 0.5,
        ElementKind::FluidPool { radius, .. } => *radius,
        ElementKind::SoftBridge { .. } => 1.0,
        ElementKind::Target { width, height, .. } => width.max(*height) * 0.5,
        ElementKind::Drain { width } => *width * 0.5,
        ElementKind::BallSpawn => BALL_RADIUS,
    }
}

pub(crate) fn rotation_handle_pos(elem: &BoardElement) -> Vec2 {
    let extent = element_extent(elem).max(0.4);
    elem.position + rotate_vec(Vec2::new(extent + 0.45, 0.0), elem.rotation)
}

pub(crate) fn screen_to_world(app: &App, settings: &EguiSettings) -> Vec2 {
    let mouse_x = app.mouse.x;
    let mouse_y = app.mouse.y;
    Vec2::new(
        mouse_x / settings.scale + settings.cam_x,
        mouse_y / settings.scale + settings.cam_y,
    )
}

pub(crate) fn snap_to_grid(settings: &EguiSettings, p: Vec2) -> Vec2 {
    if settings.snap_to_grid {
        Vec2::new(
            snap_value(p.x, settings.grid_size),
            snap_value(p.y, settings.grid_size),
        )
    } else {
        p
    }
}
