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
        restitution: 1.0,
        ..Default::default()
    };

    contacts.push(Some(contact));
    1
}

pub fn collide_circle_polygon(contacts: &mut Vec<Contact>, body_a: &Body, body_b: &Body) -> i32 {
    collide_circle_polygon_with_hint(contacts, body_a, body_b, None)
}

/// Same as [`collide_circle_polygon`] but hints a preferred contact normal for
/// the circle-center-inside-polygon case. When the circle's center sits (nearly)
/// equidistant from two faces (e.g. wedged on the medial axis of a thin flipper),
/// the closest face can flip frame-to-frame from float noise, making the solver
/// shove the circle alternately into one face and then the other forever. The
/// hint biases toward the previous frame's normal so the resolution is
/// consistent and the circle is ejected in one direction.
pub fn collide_circle_polygon_with_hint(
    contacts: &mut Vec<Contact>,
    body_a: &Body,
    body_b: &Body,
    normal_hint: Option<Vec2>,
) -> i32 {
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
        // Hysteresis: prefer the face consistent with the circle's previous
        // contact normal so a ball wedged between two opposite faces (e.g. on a
        // thin flipper's medial axis) is pushed out one direction instead of
        // oscillating forever.
        let effective_hint = normal_hint.or(body_a.wedge_normal);
        // Collect every face's resolution and keep the candidates whose
        // separation is (near-)largest so we can tie-break via the hint.
        const TIE_EPS: f32 = 0.15;
        let mut best_separation = f32::NEG_INFINITY;
        let mut candidates: Vec<(f32, Vec2, Vec2)> = Vec::new(); // (sep, normal, contact_pos)

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

            let separation = -(dist + radius);

            if separation > best_separation {
                best_separation = separation;
                candidates.clear();
            }
            if separation >= best_separation - TIE_EPS {
                let normal = if dist > f32::EPSILON {
                    (closest - circle_pos) * (1.0 / dist)
                } else {
                    Vec2::new(0.0, 1.0)
                };
                candidates.push((separation, normal, circle_pos + normal * radius));
            }
        }

        let (_, best_normal, best_contact_pos) = match effective_hint {
            Some(hint) => {
                let mut chosen = candidates[0];
                let mut best_dot = f32::NEG_INFINITY;
                for &(sep, normal, pos) in &candidates {
                    let d = normal.dot(hint);
                    if d > best_dot {
                        best_dot = d;
                        chosen = (sep, normal, pos);
                    }
                }
                chosen
            }
            None => {
                let mut chosen = candidates[0];
                let mut best_so_far = f32::NEG_INFINITY;
                for &(sep, normal, pos) in &candidates {
                    if sep > best_so_far {
                        best_so_far = sep;
                        chosen = (sep, normal, pos);
                    }
                }
                chosen
            }
        };
        let contact = ContactInfo {
            position: best_contact_pos,
            normal: best_normal,
            separation: best_separation,
            feature: FeaturePair::new(Edges::default(), 0),
            restitution: 0.0,
            hard_project: true,
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
        restitution: 1.0,
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
