use crate::arbiter::{Contact, ContactInfo, Edges, FeaturePair};
use crate::body::Body;
use crate::math_utils::Vec2;

pub fn collide_circle_circle(contacts: &mut Vec<Contact>, body_a: &Body, body_b: &Body) -> i32 {
    let diff = body_b.position - body_a.position;
    let dist_sq = diff.dot(diff);
    let radius_sum = body_a.radius + body_b.radius;

    if dist_sq > radius_sum * radius_sum {
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

    let poly = body_b.get_polygon().rotate(body_b.rotation).translate(body_b.position);
    let n = poly.get_num_vertices();

    if n == 0 {
        return 0;
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
    if dist > radius {
        return 0;
    }

    let normal = if dist > f32::EPSILON {
        (circle_pos - closest_point) * (1.0 / dist)
    } else {
        Vec2::new(0.0, 1.0)
    };

    let contact = ContactInfo {
        position: closest_point,
        normal,
        separation: dist - radius,
        feature: FeaturePair::new(Edges::default(), 0),
        ..Default::default()
    };

    contacts.push(Some(contact));
    1
}
