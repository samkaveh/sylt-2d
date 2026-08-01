use crate::body::Body;
use crate::math_utils::{Mat2x2, Vec2};

/// Result of a continuous collision test.
/// `t` is a normalized impact time in `[0.0, 1.0]` (0 == start, 1 == end).
#[derive(Clone, Copy, Debug)]
pub struct Impact {
    pub t: f32,
    pub position: Vec2,
    pub normal: Vec2,
}

/// Continuous test between two circles moving with linear velocity `v1`, `v2`
/// over a time step `dt`. Returns the earliest impact.
pub fn sweep_circle_circle(b1: &Body, b2: &Body, dt: f32) -> Option<Impact> {
    let p1 = b1.position;
    let p2 = b2.position;
    let v1 = b1.velocity;
    let v2 = b2.velocity;
    let r1 = b1.radius;
    let r2 = b2.radius;
    let r = r1 + r2;

    let rel = v1 - v2;
    // Relative displacement over the full step; `t` below is normalized in
    // [0, 1] across `dt`.
    let rel_d = rel * dt;
    let base = p1 - p2;

    let a = rel_d.dot(rel_d);
    if a < f32::EPSILON {
        return None;
    }

    let b = 2.0 * base.dot(rel_d);
    let c = base.dot(base) - r * r;

    if c > 0.0 && b > 0.0 {
        // Moving apart and already separated.
        return None;
    }

    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return None;
    }

    let sqrt_d = discriminant.sqrt();
    let t = (-b - sqrt_d) / (2.0 * a);
    let t_end = (-b + sqrt_d) / (2.0 * a);

    let t = t.max(0.0);
    if t >= 1.0 || t_end < 0.0 {
        return None;
    }

    let t_clamped = t.clamp(0.0, 1.0);
    let impact_t = t_clamped * dt;

    let p1_hit = p1 + v1 * impact_t;
    let p2_hit = p2 + v2 * impact_t;
    let diff = p1_hit - p2_hit;
    let dist = diff.length();

    if dist < f32::EPSILON {
        return None;
    }

    let normal = diff * (1.0 / dist);
    // Contact position at the surface of circle 1.
    let position = p1_hit + normal * r1;
    let _ = position;

    Some(Impact {
        t: t_clamped,
        position: p1_hit + normal * r1,
        normal,
    })
}

/// Raycast of a point `origin` moving by `delta` against a disc of `radius`
/// centered at `center` (in 2D). The normal points from the disc center toward
/// the point at impact, i.e. outward from the obstacle toward the mover.
fn ray_circle(origin: Vec2, delta: Vec2, center: Vec2, radius: f32) -> Option<(f32, Vec2, Vec2)> {
    let m = origin - center;
    let a = delta.dot(delta);
    if a < f32::EPSILON {
        // No relative motion: no sweep impact (overlap is handled discretely).
        return None;
    }
    let b = 2.0 * m.dot(delta);
    let c = m.dot(m) - radius * radius;
    if c <= 0.0 {
        // Already inside the disc; treat as overlap, not a sweep impact.
        return None;
    }
    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return None;
    }
    let t = (-b - discriminant.sqrt()) / (2.0 * a);
    if t < 0.0 || t > 1.0 {
        return None;
    }
    let hit = origin + delta * t;
    let to_hit = hit - center;
    let len = to_hit.length();
    if len < f32::EPSILON {
        return None;
    }
    let normal = to_hit * (1.0 / len);
    Some((t, hit, normal))
}

/// Raycast of a point `origin` moving by `delta` against the infinite plane of a
/// face with unit outward normal `n` passing through `plane_point`, constrained
/// to the face span along `edge` (length squared `edge_len_sq`). The normal
/// points outward from the box (toward the mover).
fn ray_face(
    origin: Vec2,
    delta: Vec2,
    plane_point: Vec2,
    n: Vec2,
    edge: Vec2,
    edge_len_sq: f32,
) -> Option<(f32, Vec2, Vec2)> {
    let denom = delta.dot(n);
    if denom.abs() < f32::EPSILON {
        return None;
    }
    let t = (plane_point - origin).dot(n) / denom;
    if t < 0.0 || t > 1.0 {
        return None;
    }
    let hit = origin + delta * t;
    // Constrain to the face segment span (projection along the original edge).
    let proj = (hit - plane_point).dot(edge) / edge_len_sq;
    if proj < 0.0 || proj > 1.0 {
        return None;
    }
    Some((t, hit, n))
}

/// Continuous test between a moving circle and a (linearly) moving convex
/// polygon (box or arbitrary polygon) over a time step `dt`. The swept circle is
/// tested against the exact rounded Minkowski sum: each face offset outward by
/// the circle radius and a disc of the circle radius at each vertex. The
/// earliest impact wins. Body velocities are assumed constant; relative motion
/// is used for the sweep.
pub fn sweep_circle_polygon(circle: &Body, poly_body: &Body, dt: f32) -> Option<Impact> {
    let r = circle.radius;
    let poly = poly_body
        .get_polygon()
        .rotate(poly_body.rotation)
        .translate(poly_body.position);
    let n = poly.get_num_vertices();
    if n < 3 {
        return None;
    }

    let origin = circle.position;
    let delta = (circle.velocity - poly_body.velocity) * dt;

    let mut best: Option<Impact> = None;
    let mut consider = |t: f32, position: Vec2, normal: Vec2| {
        match &best {
            None => best = Some(Impact { t, position, normal }),
            Some(cur) if t < cur.t => best = Some(Impact { t, position, normal }),
            _ => {}
        }
    };

    // Vertex discs: ray vs a disc of the circle radius at each polygon vertex.
    for i in 0..n {
        let v = poly.get_vertex(i as isize);
        if let Some((t, hit, nrm)) = ray_circle(origin, delta, v, r) {
            consider(t, hit, nrm);
        }
    }

    // Faces: ray vs each edge offset outward by the circle radius.
    for i in 0..n {
        let a = poly.get_vertex(i as isize);
        let b = poly.get_vertex(i as isize + 1);
        let edge = b - a;
        let len = edge.length();
        if len < f32::EPSILON {
            continue;
        }
        let nrm = Vec2::new(edge.y, -edge.x) * (1.0 / len);
        if let Some((t, hit, _)) = ray_face(origin, delta, a + nrm * r, nrm, edge, edge.dot(edge))
        {
            consider(t, hit, nrm);
        }
    }

    best
}

/// Continuous test between a moving circle and a (linearly) moving box over a
/// time step `dt`. Boxes are convex polygons, so this reuses the exact rounded
/// polygon sweep.
pub fn sweep_circle_box(circle: &Body, b2: &Body, dt: f32) -> Option<Impact> {
    sweep_circle_polygon(circle, b2, dt)
}

/// Continuous test between two (possibly rotated) boxes using conservative
/// bisection on the separating-axis penetration. Returns earliest impact.
pub fn sweep_box_box(b1: &Body, b2: &Body, dt: f32) -> Option<Impact> {
    // Use relative motion and SAT-based distance; bisection for sign change.
    let rot2 = Mat2x2::new_from_angle(b2.rotation);
    let hw1 = b1.width * 0.5;
    let hw2 = b2.width * 0.5;

    let v_rel = b1.velocity - b2.velocity;
    let w1 = b1.angular_velocity;
    let w2 = b2.angular_velocity;

    // Sample points on box edges (vertices of b1) to test distance against b2.
    let verts1_local = [
        Vec2::new(hw1.x, hw1.y),
        Vec2::new(-hw1.x, hw1.y),
        Vec2::new(-hw1.x, -hw1.y),
        Vec2::new(hw1.x, -hw1.y),
    ];

    // Distance from a point to a box (in box's local frame). Negative if inside.
    fn point_box_dist(
        p_world: Vec2,
        pos: Vec2,
        rot: Mat2x2,
        h: Vec2,
    ) -> (f32, Vec2) {
        let rot_t = rot.transpose();
        let local = rot_t * (p_world - pos);
        let dx = local.x.abs() - h.x;
        let dy = local.y.abs() - h.y;
        let pen = dx.max(dy);
        let outside = if dx > 0.0 || dy > 0.0 {
            let nx = dx.max(0.0);
            let ny = dy.max(0.0);
            let n = Vec2::new(nx, ny);
            if n.length() < f32::EPSILON {
                Vec2::new(1.0, 0.0)
            } else {
                rot * (n * (1.0 / n.length()))
            }
        } else {
            // Inside: use the separating axis with smallest penetration.
            let abs_local = local.abs();
            let axis = if abs_local.x > abs_local.y {
                Vec2::new(local.x.signum(), 0.0)
            } else {
                Vec2::new(0.0, local.y.signum())
            };
            rot * axis
        };
        (pen, outside)
    }

    // Binary search for the earliest time t in [0,1] where a vertex of b1
    // contacts (pen == 0) b2, considering both translation and rotation.
    let mut lo = 0.0f32;
    let mut hi = 1.0f32;

    // Helper to compute current penetration of b1 verts into b2.
    let eval = |t: f32| -> f32 {
        let ang1 = b1.rotation + w1 * t * dt;
        let ang2 = b2.rotation + w2 * t * dt;
        let r1 = Mat2x2::new_from_angle(ang1);
        let r2 = Mat2x2::new_from_angle(ang2);
        // Use full relative motion of b1 with respect to b2.
        let pos1 = b1.position + v_rel * t * dt;
        let mut min_pen = f32::INFINITY;
        for lv in &verts1_local {
            let p = pos1 + r1 * *lv;
            let (pen, _) = point_box_dist(p, b2.position, r2, hw2);
            if pen < min_pen {
                min_pen = pen;
            }
        }
        min_pen
    };

    let pen0 = eval(0.0);
    let pen1 = eval(1.0);

    if pen0 < 0.0 {
        // Already overlapping — treat as contact at t=0.
        let r1_0 = Mat2x2::new_from_angle(b1.rotation);
        let (pen, n) = point_box_dist(b1.position + r1_0 * verts1_local[0], b2.position, rot2, hw2);
        let _ = pen;
        return Some(Impact {
            t: 0.0,
            position: b1.position + r1_0 * verts1_local[0],
            normal: n,
        });
    }

    if pen0 >= 0.0 && pen1 >= 0.0 {
        // No sign change; could still graze. Cheap check using relative motion.
        // Find min penetration over the interval via sampling. If never <= 0, no impact.
        let steps = 8;
        let mut min_pen = pen0;
        for s in 1..=steps {
            let t = s as f32 / steps as f32;
            let pen = eval(t);
            if pen < min_pen {
                min_pen = pen;
            }
        }
        if min_pen > 0.0 {
            return None;
        }
        // There may be two crossings; binary-search the first one.
        lo = 0.0;
        hi = 1.0;
    }

    // Find first sign change via bisection.
    let eps = 1e-5;
    for _ in 0..40 {
        let mid = (lo + hi) * 0.5;
        if eval(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
        if hi - lo < eps {
            break;
        }
    }

    if hi >= 1.0 {
        return None;
    }

    let t = hi;
    let ang1 = b1.rotation + w1 * t * dt;
    let ang2 = b2.rotation + w2 * t * dt;
    let r1 = Mat2x2::new_from_angle(ang1);
    let r2 = Mat2x2::new_from_angle(ang2);
    let pos1 = b1.position + v_rel * t * dt;
    let mut best_pen = f32::INFINITY;
    let mut best = None;
    for lv in &verts1_local {
        let p = pos1 + r1 * *lv;
        let (pen, n) = point_box_dist(p, b2.position, r2, hw2);
        if pen < best_pen {
            best_pen = pen;
            best = Some((p, n));
        }
    }
    let (pos, n) = best?;
    Some(Impact {
        t,
        position: pos,
        normal: n,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn circle(pos: Vec2, vel: Vec2, radius: f32, mass: f32) -> Body {
        let mut b = Body::new_circle(radius, mass);
        b.position = pos;
        b.velocity = vel;
        b
    }

    fn boxy(pos: Vec2, w: Vec2, mass: f32) -> Body {
        let mut b = Body::new(w, mass);
        b.position = pos;
        b
    }

    #[test]
    fn sweep_circle_circle_hits() {
        // Circle moving right towards a static circle ahead.
        let a = circle(Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), 1.0, 1.0);
        let b = circle(Vec2::new(5.0, 0.0), Vec2::new(0.0, 0.0), 1.0, f32::MAX);
        let impact = sweep_circle_circle(&a, &b, 1.0).unwrap();
        // Radii sum = 2.0, relative speed 10 over dt=1 -> distance closed 10.
        // Impact when centers are 2.0 apart: t = 3.0/10 = 0.3.
        assert!((impact.t - 0.299).abs() < 0.02, "t={}", impact.t);
        // Normal points from b toward a (leftward), i.e. -x.
        assert!(impact.normal.x < -0.99);
    }

    #[test]
    fn sweep_circle_circle_misses() {
        // Circle passes too far above.
        let a = circle(Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), 1.0, 1.0);
        let b = circle(Vec2::new(5.0, 5.0), Vec2::new(0.0, 0.0), 1.0, f32::MAX);
        assert!(sweep_circle_circle(&a, &b, 1.0).is_none());
    }
}
