use crate::arbiter::{Contact, ContactInfo, Edges, FeaturePair};
use crate::body::Body;
use crate::math_utils::Vec2;

/// Contact emission tolerance. Contacts are generated when surfaces are within
/// this distance of touching so that a body landing exactly on a surface still
/// produces a manifold (avoids float-rounding gaps that let the solver ignore
/// the impact).
const CONTACT_SLOP: f32 = 0.01;

pub fn collide_circle_circle(contacts: &mut Vec<Contact>, body_a: &Body, body_b: &Body) -> i32 {
    let diff = body_b.position - body_a.position;
    let dist_sq = diff.dot(diff);
    let radius_sum = body_a.radius + body_b.radius;

    if dist_sq > (radius_sum + CONTACT_SLOP) * (radius_sum + CONTACT_SLOP) {
        return 0;
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > f32::EPSILON {
        diff * (1.0 / dist)
    } else {
        Vec2::new(0.0, 1.0)
    };

    let contact = ContactInfo {
        position: body_a.position + normal * body_a.radius,
        normal,
        separation: dist - radius_sum,
        feature: FeaturePair::new(Edges::default(), 0),
        ..Default::default()
    };

    contacts.push(Some(contact));
    1
}

pub fn collide_circle_polygon(contacts: &mut Vec<Contact>, body_a: &Body, body_b: &Body) -> i32 {
    let circle_pos = body_a.position;
    let radius = body_a.radius;

    let poly = body_b
        .get_polygon()
        .rotate(body_b.rotation)
        .translate(body_b.position);
    let n = poly.get_num_vertices();

    if n == 0 {
        return 0;
    }

    let mut inside = true;
    for i in 0..n {
        let v1 = poly.get_vertex(i as isize);
        let v2 = poly.get_vertex((i + 1) as isize);
        let edge = v2 - v1;
        let to_point = circle_pos - v1;
        let cross = edge.x * to_point.y - edge.y * to_point.x;
        if cross <= 0.0 {
            inside = false;
            break;
        }
    }

    if inside {
        let mut best_separation = f32::NEG_INFINITY;
        let mut best_normal = Vec2::new(0.0, 0.0);
        let mut best_contact_pos = Vec2::new(0.0, 0.0);

        for i in 0..n {
            let v1 = poly.get_vertex(i as isize);
            let v2 = poly.get_vertex((i + 1) as isize);
            let edge = v2 - v1;
            let edge_len_sq = edge.dot(edge);
            if edge_len_sq < f32::EPSILON {
                continue;
            }

            let t = ((circle_pos - v1).dot(edge) / edge_len_sq).clamp(0.0, 1.0);
            let closest = v1 + edge * t;
            let diff = circle_pos - closest;
            let dist = diff.length();

            let edge_len = edge_len_sq.sqrt();
            let outward = Vec2::new(edge.y / edge_len, -edge.x / edge_len);

            let separation = -(dist + radius);

            if separation > best_separation {
                best_separation = separation;
                best_normal = outward;
                best_contact_pos = circle_pos + outward * radius;
            }
        }

        let contact = ContactInfo {
            position: best_contact_pos,
            normal: best_normal,
            separation: best_separation,
            feature: FeaturePair::new(Edges::default(), 0),
            ..Default::default()
        };

        contacts.push(Some(contact));
        return 1;
    }

    let mut min_dist = f32::MAX;
    let mut closest_point = Vec2::new(0.0, 0.0);

    for i in 0..n {
        let edge_start = poly.get_vertex(i as isize);
        let edge_end = poly.get_vertex((i + 1) as isize);
        let edge = edge_end - edge_start;

        let t = ((circle_pos - edge_start).dot(edge)) / edge.dot(edge);
        let t_clamped = t.clamp(0.0, 1.0);
        let point = edge_start + edge * t_clamped;

        let diff = circle_pos - point;
        let dist_sq = diff.dot(diff);

        if dist_sq < min_dist {
            min_dist = dist_sq;
            closest_point = point;
        }
    }

    let dist = min_dist.sqrt();
    if dist > radius + CONTACT_SLOP {
        return 0;
    }

    let normal = if dist > f32::EPSILON {
        (closest_point - circle_pos) * (1.0 / dist)
    } else {
        Vec2::new(0.0, 1.0)
    };

    let contact = ContactInfo {
        position: circle_pos + normal * radius,
        normal,
        separation: dist - radius,
        feature: FeaturePair::new(Edges::default(), 0),
        ..Default::default()
    };

    contacts.push(Some(contact));
    1
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math_utils::Vec2;

    fn make_circle(pos: Vec2, radius: f32) -> Body {
        let mut b = Body::new_circle(radius, 1.0);
        b.position = pos;
        b
    }

    fn make_box(pos: Vec2, half_w: f32, half_h: f32) -> Body {
        let mut b = Body::new(Vec2::new(half_w * 2.0, half_h * 2.0), 1.0);
        b.position = pos;
        b
    }

    #[test]
    fn circle_outside_no_collision() {
        let circle = make_circle(Vec2::new(10.0, 10.0), 1.0);
        let bx = make_box(Vec2::new(0.0, 0.0), 1.0, 1.0);
        let mut contacts = Vec::new();
        let n = collide_circle_polygon(&mut contacts, &circle, &bx);
        assert_eq!(n, 0);
    }

    #[test]
    fn circle_outside_overlapping_edge() {
        let circle = make_circle(Vec2::new(1.5, 0.0), 1.0);
        let bx = make_box(Vec2::new(0.0, 0.0), 1.0, 1.0);
        let mut contacts = Vec::new();
        let n = collide_circle_polygon(&mut contacts, &circle, &bx);
        assert_eq!(n, 1);
        let c = contacts[0].as_ref().unwrap();
        assert!(
            c.separation < 0.0,
            "should be overlapping, sep={}",
            c.separation
        );
        // Normal now points from circle toward polygon (leftward)
        assert!(
            c.normal.x < -0.9,
            "normal should point left toward polygon, got {:?}",
            c.normal
        );
    }

    #[test]
    fn circle_center_inside_box() {
        let circle = make_circle(Vec2::new(0.0, 0.0), 1.0);
        let bx = make_box(Vec2::new(0.0, 0.0), 2.0, 2.0);
        let mut contacts = Vec::new();
        let n = collide_circle_polygon(&mut contacts, &circle, &bx);
        assert_eq!(n, 1, "should detect collision when circle is inside box");
        let c = contacts[0].as_ref().unwrap();
        assert!(
            c.separation < 0.0,
            "separation should be negative, got {}",
            c.separation
        );
        let sep = c.separation;
        assert!(sep < -1.9, "deep penetration expected, sep={}", sep);
    }

    #[test]
    fn circle_deeply_inside_box_normal_direction() {
        let circle = make_circle(Vec2::new(0.5, 0.3), 1.0);
        let bx = make_box(Vec2::new(0.0, 0.0), 3.0, 3.0);
        let mut contacts = Vec::new();
        let n = collide_circle_polygon(&mut contacts, &circle, &bx);
        assert_eq!(n, 1);
        let c = contacts[0].as_ref().unwrap();
        assert!(
            c.separation < -1.0,
            "should have significant penetration, sep={}",
            c.separation
        );
        assert!(
            c.normal.x > 0.0,
            "normal should point toward nearest wall, got {:?}",
            c.normal
        );
    }

    #[test]
    fn circle_inside_box_normal_pushes_out_right() {
        let circle = make_circle(Vec2::new(1.8, 0.0), 1.0);
        let bx = make_box(Vec2::new(0.0, 0.0), 2.0, 2.0);
        let mut contacts = Vec::new();
        let n = collide_circle_polygon(&mut contacts, &circle, &bx);
        assert_eq!(n, 1);
        let c = contacts[0].as_ref().unwrap();
        assert!(
            c.normal.x > 0.0,
            "normal should point right (outward), got {:?}",
            c.normal
        );
        assert!(
            c.normal.y.abs() < 0.1,
            "normal should be mostly horizontal, got {:?}",
            c.normal
        );
    }

    #[test]
    fn circle_inside_box_normal_pushes_out_top() {
        let circle = make_circle(Vec2::new(0.0, 1.8), 1.0);
        let bx = make_box(Vec2::new(0.0, 0.0), 2.0, 2.0);
        let mut contacts = Vec::new();
        let n = collide_circle_polygon(&mut contacts, &circle, &bx);
        assert_eq!(n, 1);
        let c = contacts[0].as_ref().unwrap();
        assert!(
            c.normal.y > 0.0,
            "normal should point up (outward), got {:?}",
            c.normal
        );
        assert!(
            c.normal.x.abs() < 0.1,
            "normal should be mostly vertical, got {:?}",
            c.normal
        );
    }

    #[test]
    fn circle_outside_touching_vertex() {
        let circle = make_circle(Vec2::new(2.0, 1.0), 1.0);
        let bx = make_box(Vec2::new(0.0, 0.0), 1.0, 1.0);
        let mut contacts = Vec::new();
        let n = collide_circle_polygon(&mut contacts, &circle, &bx);
        assert_eq!(n, 1);
        let c = contacts[0].as_ref().unwrap();
        assert!(
            c.separation <= 0.0,
            "should be touching or overlapping, sep={}",
            c.separation
        );
    }

    #[test]
    fn circle_center_on_polygon_edge() {
        let circle = make_circle(Vec2::new(1.0, 0.0), 1.0);
        let bx = make_box(Vec2::new(0.0, 0.0), 1.0, 1.0);
        let mut contacts = Vec::new();
        let n = collide_circle_polygon(&mut contacts, &circle, &bx);
        assert!(n <= 1);
    }

    #[test]
    fn circle_box_no_sticking_over_multiple_steps() {
        let circle = make_circle(Vec2::new(3.99, 4.98), 1.0);
        let bx = make_box(Vec2::new(4.0, 4.0), 1.0, 1.0);
        let mut contacts = Vec::new();
        let n = collide_circle_polygon(&mut contacts, &circle, &bx);
        assert_eq!(n, 1);
        let c = contacts[0].as_ref().unwrap();
        assert!(
            c.normal.y > 0.5,
            "normal should point up toward top wall, got {:?}",
            c.normal
        );
        assert!(
            c.separation < 0.0,
            "should be penetrating, sep={}",
            c.separation
        );
        let expected_sep = -(0.02 + 1.0);
        assert!(
            (c.separation - expected_sep).abs() < 0.01,
            "sep should be ~{}, got {}",
            expected_sep,
            c.separation
        );
    }
}
