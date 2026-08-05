use crate::{
    arbiter::{Contact, ContactInfo, Edges, FeaturePair},
    body::{Body, ConvexPolygon},
    math_utils::Vec2,
};

/// Determines which side of the line the polygon is on.
///
/// # Arguments
/// * `polygon` - The polygon to check.
/// * `p` - A point on the line.
/// * `d` - The direction vector of the line.
///
/// # Returns
/// * `1` if all vertices of the polygon are on the positive side.
/// * `-1` if all vertices are on the negative side.
/// * `0` if the polygon intersects or lies on the line.
fn which_side(polygon: &ConvexPolygon, p: Vec2, d: Vec2) -> i32 {
    let mut pos_count = 0;
    let mut neg_count = 0;

    for i in 0..polygon.get_num_vertices() {
        let vertex = polygon.get_vertex(i as isize);
        let t = d.dot(vertex - p);
        if t > 0.0 {
            pos_count += 1;
        } else if t < 0.0 {
            neg_count += 1;
        }
        if pos_count > 0 && neg_count > 0 {
            return 0; // Line splits the polygon
        }
    }

    if pos_count > 0 {
        1 // All vertices are on the positive side
    } else {
        -1 // All vertices are on the negative side
    }
}

/// Tests for intersection between two convex polygons using the Separating Axis Theorem.
///
/// # Arguments
/// * `c0` - The first convex polygon.
/// * `c1` - The second convex polygon.
///
/// # Returns
/// * `true` if the polygons intersect.
/// * `false` if they are separated.
fn test_intersection(c0: &ConvexPolygon, c1: &ConvexPolygon) -> bool {
    // Test edges of c0 for separation
    for i0 in 0..c0.get_num_vertices() {
        let i1 = (i0 + 1) % c0.get_num_vertices();
        let p = c0.get_vertex(i1 as isize);
        let d = c0.get_normal(i0 as isize);
        if which_side(c1, p, d) > 0 {
            return false; // c1 is entirely on the positive side of the line
        }
    }

    // Test edges of c1 for separation
    for i0 in 0..c1.get_num_vertices() {
        let i1 = (i0 + 1) % c1.get_num_vertices();
        let p = c1.get_vertex(i1 as isize);
        let d = c1.get_normal(i0 as isize);
        if which_side(c0, p, d) > 0 {
            return false; // c0 is entirely on the positive side of the line
        }
    }

    true // No separating axis found; polygons intersect
}

/// Clips a polygon against a given edge of another polygon.
///
/// # Arguments
/// * `polygon` - The polygon to be clipped.
/// * `clip_polygon` - The polygon to use for clipping.
///
/// # Returns
/// A list of clipped points.
pub fn clip_polygon(polygon: &ConvexPolygon, clip_polygon: &ConvexPolygon) -> Vec<(Vec2, Vec2)> {
    let mut polygon: ConvexPolygon = ConvexPolygon::new(polygon.get_vertices());

    // This will store the final clipped vertices along with their normals
    let mut clipped: Vec<(Vec2, Vec2)> = Vec::new();

    // Iterate over all edges of the clipping polygon
    for j in 0..clip_polygon.get_num_vertices() {
        let edge_start = clip_polygon.get_vertex(j as isize);
        let edge_normal = clip_polygon.get_normal(j as isize);

        // Temporary storage for the current iteration
        let mut current_clipped: Vec<(Vec2, Vec2)> = Vec::new();

        let n = polygon.get_num_vertices();
        for i in 0..n {
            let current = polygon.get_vertex(i as isize);
            let next = polygon.get_vertex((i + 1) as isize);

            // Distances from the current and next points to the clipping plane
            let dist_current = edge_normal.dot(current - edge_start) / edge_normal.length();
            let dist_next = edge_normal.dot(next - edge_start) / edge_normal.length();

            if dist_current <= 0.0 {
                // Current point is inside or on the plane
                current_clipped.push((current, edge_normal));
            }

            if dist_current * dist_next < 0.0 {
                // Edge intersects the plane; compute intersection point
                let interp = dist_current / (dist_current - dist_next);
                let intersection = current + (next - current) * interp;
                current_clipped.push((intersection, edge_normal));
            }
        }

        // Prepare for next iteration
        let clipped_vertices: Vec<Vec2> = current_clipped.iter().map(|tuple| tuple.0).collect();
        polygon = ConvexPolygon::new(clipped_vertices);

        clipped = current_clipped;
    }

    // Assign normals to clipped vertices based on closest edge of the clipping polygon
    let mut final_clipped = Vec::new();
    for (vertex, _) in clipped {
        let mut closest_normal = Vec2::new(0.0, 0.0);
        let mut min_distance = f32::MAX;

        for j in 0..clip_polygon.get_num_vertices() {
            let edge_start = clip_polygon.get_vertex(j as isize);
            let edge_end = clip_polygon.get_vertex((j + 1) as isize);

            let edge = edge_end - edge_start;
            let mut normal = Vec2::new(-edge.y, edge.x); // Outward-facing normal
            normal = normal * (1.0 / normal.length());
            let to_point = vertex - edge_start;
            let distance = (to_point.dot(normal)).abs();

            if distance < min_distance {
                min_distance = distance;
                closest_normal = normal;
            }
        }

        final_clipped.push((vertex, closest_normal));
    }

    final_clipped
}
/// Finds contact points between two intersecting convex polygons.
///
/// # Arguments
/// * `c0` - The first convex polygon (reference).
/// * `c1` - The second convex polygon (incident).
///
/// # Returns
/// A vector of contact points, where each contact point includes:
/// - `Point`: The position of the contact point.
/// - `Point`: The normal at the contact point.
// Find contact points and store them in the Contact type
fn find_contact_points(c0: &ConvexPolygon, c1: &ConvexPolygon) -> Vec<Contact> {
    let mut result: Vec<Contact> = Vec::new();
    // Clip the current contact points against this edge
    let clipped = clip_polygon(c0, c1);

    // If no points remain, polygons are not intersecting
    if clipped.is_empty() {
        return Vec::new();
    }

    // Process each contact point and store the contact info
    for (point, normal) in &clipped {
        let relative_position = *point;
        let separation = relative_position.dot(*normal);

        // Create FeaturePair (assuming edges is a pair of edge indices)
        let feature = FeaturePair::new(Edges::default(), 0); // Replace 0 with appropriate value

        let contact_info = ContactInfo {
            position: *point,
            normal: *normal,
            separation: separation * 0.001,
            feature,
            restitution: 1.0,
            ..Default::default()
        };

        // Add the contact info to the result vector
        result.push(Some(contact_info));
    }
    result
}
pub fn collide_polygons(contacts: &mut Vec<Contact>, b1: &Body, b2: &Body) -> i32 {
    let c0 = b1.get_polygon().rotate(b1.rotation).translate(b1.position);
    let c1 = b2.get_polygon().rotate(b2.rotation).translate(b2.position);
    if test_intersection(&c0, &c1) {
        *contacts = find_contact_points(&c0, &c1);
    }

    contacts.len() as i32
}
