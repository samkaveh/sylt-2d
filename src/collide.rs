use crate::arbiter::{Contact, ContactInfo, EdgeNumbers, Edges, FeaturePair};
use crate::body::Body;
use crate::math_utils::{Mat2x2, Vec2};

// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3

#[derive(PartialEq)]
enum Axis {
    FaceAX,
    FaceAY,
    FaceBX,
    FaceBY,
}

#[derive(Clone, Copy)]
pub struct ClipVertex {
    fp: FeaturePair,
    pub v: Vec2,
}

impl Default for ClipVertex {
    fn default() -> Self {
        Self {
            fp: FeaturePair::new(Edges::default(), 0),
            v: Vec2::new(0.0, 0.0),
        }
    }
}

fn flip(fp: &mut FeaturePair) {
    std::mem::swap(&mut fp.edges.in_edge_1, &mut fp.edges.in_edge_2);
    std::mem::swap(&mut fp.edges.out_edge_1, &mut fp.edges.out_edge_2);
}

fn clip_segment_to_line(
    v_out: &mut [ClipVertex; 2],
    v_in: [ClipVertex; 2],
    normal: &Vec2,
    offset: f32,
    clip_edge: EdgeNumbers,
) -> usize {
    let mut num_out: usize = 0;

    // Calculate the distance of end points to the line
    let distance_0 = normal.dot(v_in[0].v) - offset;
    let distance_1 = normal.dot(v_in[1].v) - offset;

    // If the points are behind the plane
    if distance_0 <= 0.0 {
        v_out[num_out] = v_in[0];
        num_out += 1;
    }
    if distance_1 <= 0.0 {
        v_out[num_out] = v_in[1];
        num_out += 1;
    }

    // If the points are on different sides of the plane
    if distance_0 * distance_1 < 0.0 {
        // Find intersection point of edge and plane
        let interp = distance_0 / (distance_0 - distance_1);
        v_out[num_out].v = v_in[0].v + (v_in[1].v - v_in[0].v) * interp;

        // Set feature points based on which point is in front of the plane
        if distance_0 > 0.0 {
            v_out[num_out].fp = v_in[0].fp;
            v_out[num_out].fp.edges.in_edge_1 = clip_edge;
            v_out[num_out].fp.edges.in_edge_2 = EdgeNumbers::NoEdge;
        } else {
            v_out[num_out].fp = v_in[1].fp;
            v_out[num_out].fp.edges.out_edge_1 = clip_edge;
            v_out[num_out].fp.edges.out_edge_2 = EdgeNumbers::NoEdge;
        }
        num_out += 1;
    }

    num_out
}

fn compute_incident_edge(h: &Vec2, pos: &Vec2, rot: &Mat2x2, normal: &Vec2) -> [ClipVertex; 2] {
    // The normal is from the reference box. Convert it
    // to the incident boxe's frame and flip sign.
    let rot_t = rot.transpose();
    let n = -(rot_t * *normal);
    let abs_n = n.abs();
    let mut c1 = ClipVertex::default();
    let mut c2 = ClipVertex::default();

    if abs_n.x > abs_n.y {
        if n.x.signum() > 0.0 {
            c1.v = Vec2::new(h.x, -h.y);
            c1.fp.edges.in_edge_2 = EdgeNumbers::Edge3;
            c1.fp.edges.out_edge_2 = EdgeNumbers::Edge4;

            c2.v = Vec2::new(h.x, h.y);
            c2.fp.edges.in_edge_2 = EdgeNumbers::Edge4;
            c2.fp.edges.out_edge_2 = EdgeNumbers::Edge1;
        } else {
            c1.v = Vec2::new(-h.x, h.y);
            c1.fp.edges.in_edge_2 = EdgeNumbers::Edge1;
            c1.fp.edges.out_edge_2 = EdgeNumbers::Edge2;

            c2.v = Vec2::new(-h.x, -h.y);
            c2.fp.edges.in_edge_2 = EdgeNumbers::Edge2;
            c2.fp.edges.out_edge_2 = EdgeNumbers::Edge3;
        }
    } else if n.y.signum() > 0.0 {
        c1.v = Vec2::new(h.x, h.y);
        c1.fp.edges.in_edge_2 = EdgeNumbers::Edge4;
        c1.fp.edges.out_edge_2 = EdgeNumbers::Edge1;

        c2.v = Vec2::new(-h.x, h.y);
        c2.fp.edges.in_edge_2 = EdgeNumbers::Edge1;
        c2.fp.edges.out_edge_2 = EdgeNumbers::Edge2;
    } else {
        c1.v = Vec2::new(-h.x, -h.y);
        c1.fp.edges.in_edge_2 = EdgeNumbers::Edge2;
        c1.fp.edges.out_edge_2 = EdgeNumbers::Edge3;

        c2.v = Vec2::new(h.x, -h.y);
        c2.fp.edges.in_edge_2 = EdgeNumbers::Edge3;
        c2.fp.edges.out_edge_2 = EdgeNumbers::Edge4;
    }

    c1.v = *pos + (*rot * c1.v);
    c2.v = *pos + (*rot * c2.v);
    [c1, c2]
}

pub fn collide(contacts: &mut Vec<Contact>, body_a: &Body, body_b: &Body) -> i32 {
    let h_a = body_a.width * 0.5;
    let h_b = body_b.width * 0.5;

    let pos_a = body_a.position;
    let pos_b = body_b.position;
    let rot_a = Mat2x2::new_from_angle(body_a.rotation);
    let rot_b = Mat2x2::new_from_angle(body_b.rotation);

    let rot_a_t = rot_a.transpose();
    let rot_b_t = rot_b.transpose();

    let d_p = pos_b - pos_a;
    let d_a = rot_a_t * d_p; // distance between two centers in frame of A
    let d_b = rot_b_t * d_p;

    let c = rot_a_t * rot_b;
    let abs_c = c.abs();
    let abs_c_t = c.abs().transpose();

    // Check if the boxes perotrude in one another
    let face_a = d_a.abs() - h_a - abs_c * h_b; // d_a - half width of A and half width of B in frame of A (how much is the box B protruding in A in frame of A)
    if face_a.x > 0.0 || face_a.y > 0.0 {
        return 0;
    };
    let face_b = d_b.abs() - h_b - abs_c_t * h_a;
    if face_b.x > 0.0 || face_b.y > 0.0 {
        return 0;
    };

    // Find the axis with smallest seperation
    let mut axis = Axis::FaceAX;
    let mut separation = face_a.x;
    let mut normal = if d_a.x > 0.0 { rot_a.col1 } else { -rot_a.col1 };

    // Tolerances
    const RELATIVE_TOL: f32 = 0.95;
    const ABSOLUTE_TOL: f32 = 0.01;

    // Box A faces
    if face_a.y > RELATIVE_TOL * separation + ABSOLUTE_TOL * h_a.y {
        axis = Axis::FaceAY;
        separation = face_a.y;
        normal = if d_a.y > 0.0 { rot_a.col2 } else { -rot_a.col2 };
    }

    // Box B faces
    if face_b.x > RELATIVE_TOL * separation + ABSOLUTE_TOL * h_b.x {
        axis = Axis::FaceBX;
        separation = face_b.x;
        normal = if d_b.x > 0.0 { rot_b.col1 } else { -rot_b.col1 };
    }

    if face_b.y > RELATIVE_TOL * separation + ABSOLUTE_TOL * h_b.y {
        axis = Axis::FaceBY;
        normal = if d_b.y > 0.0 { rot_b.col2 } else { -rot_b.col2 };
    }

    let front_normal;
    let side_normal;
    let neg_side;
    let pos_side;
    let neg_edge;
    let pos_edge;
    let front;

    let incident_edge = match axis {
        Axis::FaceAX => {
            front_normal = normal;
            front = pos_a.dot(front_normal) + h_a.x;
            side_normal = rot_a.col2;
            let side = pos_a.dot(side_normal);
            neg_side = -side + h_a.y;
            pos_side = side + h_a.y;
            neg_edge = EdgeNumbers::Edge3;
            pos_edge = EdgeNumbers::Edge1;
            compute_incident_edge(&h_b, &pos_b, &rot_b, &front_normal)
        }
        Axis::FaceAY => {
            front_normal = normal;
            front = pos_a.dot(front_normal) + h_a.y;
            side_normal = rot_a.col1;
            let side = pos_a.dot(side_normal);
            neg_side = -side + h_a.x;
            pos_side = side + h_a.x;
            neg_edge = EdgeNumbers::Edge2;
            pos_edge = EdgeNumbers::Edge4;
            compute_incident_edge(&h_b, &pos_b, &rot_b, &front_normal)
        }
        Axis::FaceBX => {
            front_normal = -normal;
            front = pos_b.dot(front_normal) + h_b.x;
            side_normal = rot_b.col2;
            let side = pos_b.dot(side_normal);
            neg_side = -side + h_b.y;
            pos_side = side + h_b.y;
            neg_edge = EdgeNumbers::Edge3;
            pos_edge = EdgeNumbers::Edge1;
            compute_incident_edge(&h_a, &pos_a, &rot_a, &front_normal)
        }
        Axis::FaceBY => {
            front_normal = -normal;
            front = pos_b.dot(front_normal) + h_b.y;
            side_normal = rot_b.col1;
            let side = pos_b.dot(side_normal);
            neg_side = -side + h_b.x;
            pos_side = side + h_b.x;
            neg_edge = EdgeNumbers::Edge2;
            pos_edge = EdgeNumbers::Edge4;
            compute_incident_edge(&h_a, &pos_a, &rot_a, &front_normal)
        }
    };
    let mut clip_points1 = [ClipVertex::default(), ClipVertex::default()];
    let mut clip_points2 = [ClipVertex::default(), ClipVertex::default()];

    let mut np = clip_segment_to_line(
        &mut clip_points1,
        incident_edge,
        &(-side_normal),
        neg_side,
        neg_edge,
    );
    if np < 2 {
        return 0;
    };

    np = clip_segment_to_line(
        &mut clip_points2,
        clip_points1,
        &(side_normal),
        pos_side,
        pos_edge,
    );
    if np < 2 {
        return 0;
    };
    let mut num_contacts = 0;

    for clip_point in &mut clip_points2 {
        let separation = front_normal.dot(clip_point.v) - front;

        if separation <= 0.0 {
            if axis == Axis::FaceBX || axis == Axis::FaceBY {
                flip(&mut clip_point.fp);
            }
            let contact = ContactInfo {
                separation,
                normal,
                position: clip_point.v - front_normal * separation,
                feature: clip_point.fp,
                ..ContactInfo::default()
            };
            contacts.push(Some(contact));
            num_contacts += 1;
        }
    }
    num_contacts
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::draw::{add_box, add_line, draw_collision_result, draw_grid, get_styles, make_grid};
    use crate::math_utils::Vec2;

    #[test]
    fn test_no_overlap() {
        let styles = get_styles();
        let mut grid = make_grid(30);

        let pos_a = Vec2::new(10.0, 1.0);
        let pos_b = Vec2::new(15.0, 5.0);
        let mut box_a = Body::new(Vec2::new(1.0, 1.0), 1.0);
        box_a.position = pos_a;
        let mut box_b = Body::new(Vec2::new(1.0, 1.0), 1.0);
        box_b.position = pos_b;

        // Draw the boxes
        add_box(
            &mut grid,
            pos_a,
            box_a.width,
            box_a.rotation,
            'A',
            styles[4],
        );
        add_box(
            &mut grid,
            pos_b,
            box_b.width,
            box_b.rotation,
            'B',
            styles[6],
        );

        let rot_a = Mat2x2::new_from_angle(box_a.rotation);
        let rot_b = Mat2x2::new_from_angle(box_b.rotation);

        let rot_a_t = rot_a.transpose();
        let rot_b_t = rot_b.transpose();

        let d_p = pos_b - pos_a;
        let d_a = rot_a_t * d_p;
        let d_b = rot_b_t * d_p;

        let c = rot_a_t * rot_b;
        let abs_c = c.abs();
        let abs_c_t = c.abs().transpose();

        let face_a = d_a.abs() - (box_a.width * 0.5) - abs_c * (box_b.width * 0.5);
        let face_b = d_b.abs() - (box_b.width * 0.5) - abs_c_t * (box_a.width * 0.5);

        add_line(&mut grid, Vec2::new(0.0, 0.0), d_p, '.', styles[7]);
        add_line(&mut grid, Vec2::new(0.0, 0.0), d_a, '*', styles[4]);
        add_line(&mut grid, Vec2::new(0.0, 0.0), d_b, '@', styles[6]);
        // Perform collision detection
        let mut contacts = Vec::new();
        let num_contacts = collide(&mut contacts, &box_a, &box_b);
        println!("{:?}", contacts);
        draw_collision_result(&mut grid, &contacts);
        // Draw the grid
        add_line(&mut grid, Vec2::new(0.0, 0.0), face_b, '^', styles[1]);
        add_line(&mut grid, Vec2::new(0.0, 0.0), face_a, '+', styles[7]);
        draw_grid(&mut grid);

        // Assertions to ensure no intersection
        assert_eq!(
            num_contacts, 0,
            "Expected no contacts, but found {}",
            num_contacts
        );
    }

    #[test]
    fn test_overlap() {
        let styles = get_styles();
        let mut grid = make_grid(30);

        // Define overlapping boxes
        let pos_a = Vec2::new(11.0, 3.0);
        let pos_b = Vec2::new(12., 2.);
        let mut box_a = Body::new(Vec2::new(2.0, 2.0), 1.0);
        box_a.position = pos_a;
        let mut box_b = Body::new(Vec2::new(2.0, 2.0), 1.0);
        box_b.position = pos_b;

        // Draw the boxes
        add_box(
            &mut grid,
            pos_a,
            box_a.width,
            box_a.rotation,
            'A',
            styles[4],
        );
        add_box(
            &mut grid,
            pos_b,
            box_b.width,
            box_b.rotation,
            'B',
            styles[6],
        );
        let rot_a = Mat2x2::new_from_angle(box_a.rotation);
        let rot_b = Mat2x2::new_from_angle(box_b.rotation);

        let rot_a_t = rot_a.transpose();
        let rot_b_t = rot_b.transpose();

        let d_p = pos_b - pos_a;
        let d_a = rot_a_t * d_p;
        let d_b = rot_b_t * d_p;

        let c = rot_a_t * rot_b;
        let abs_c = c.abs();
        let abs_c_t = c.abs().transpose();

        let face_a = d_a.abs() - (box_a.width * 0.5) - abs_c * (box_b.width * 0.5);
        let face_b = d_b.abs() - (box_b.width * 0.5) - abs_c_t * (box_a.width * 0.5);

        add_line(&mut grid, Vec2::new(0.0, 0.0), d_p, '.', styles[7]);
        add_line(&mut grid, Vec2::new(0.0, 0.0), d_a, '*', styles[4]);
        add_line(&mut grid, Vec2::new(0.0, 0.0), d_b, '@', styles[6]);

        // Perform collision detection
        let mut contacts = Vec::new();
        let num_contacts = collide(&mut contacts, &box_a, &box_b);
        println!("{:?}", contacts);
        println!("\x1b[2J");

        draw_collision_result(&mut grid, &contacts);
        // Draw the grid
        add_line(&mut grid, Vec2::new(0.0, 0.0), face_b, '^', styles[1]);
        add_line(&mut grid, Vec2::new(0.0, 0.0), face_a, '+', styles[7]);
        draw_grid(&mut grid);

        // Assertions to ensure intersection
        assert!(
            num_contacts > 0,
            "Expected contacts, but found {}",
            num_contacts
        );
    }

    #[test]
    fn test_overlap_at_angle() {
        let styles = get_styles();
        let mut grid = make_grid(30);

        // Define overlapping boxes at an angle
        let pos_a = Vec2::new(12.0, 0.0);
        let pos_b = Vec2::new(15.5, 1.0);
        let mut box_a = Body::new(Vec2::new(4.0, 4.0), 1.0);
        box_a.position = pos_a;
        box_a.rotation = 45.0_f32.to_radians();

        let mut box_b = Body::new(Vec2::new(4.0, 4.0), 1.0);
        box_b.position = pos_b;
        box_b.rotation = -45.0_f32.to_radians();
        // Draw the boxes
        add_box(
            &mut grid,
            pos_a,
            box_a.width,
            box_a.rotation,
            'A',
            styles[4],
        );
        add_box(
            &mut grid,
            pos_b,
            box_b.width,
            box_b.rotation,
            'B',
            styles[6],
        );
        let rot_a = Mat2x2::new_from_angle(box_a.rotation);
        let rot_b = Mat2x2::new_from_angle(box_b.rotation);

        let rot_a_t = rot_a.transpose();
        let rot_b_t = rot_b.transpose();

        let d_p = pos_b - pos_a;
        let d_a = rot_a_t * d_p; // rotated in A frame
        let d_b = rot_b_t * d_p; // rotated in B frame

        let c = rot_a_t * rot_b;
        let abs_c = c.abs();
        let abs_c_t = c.abs().transpose();

        let face_a = d_a.abs() - (box_a.width * 0.5) - abs_c * (box_b.width * 0.5);
        let face_b = d_b.abs() - (box_b.width * 0.5) - abs_c_t * (box_a.width * 0.5);

        // Perform collision detection
        let mut contacts = Vec::new();
        let num_contacts = collide(&mut contacts, &box_a, &box_b);
        println!("{:?}", contacts);
        println!("\x1b[2J");

        draw_collision_result(&mut grid, &contacts);
        // Draw the grid
        add_line(&mut grid, Vec2::new(0.0, 0.0), d_p, '.', styles[7]);
        add_line(&mut grid, Vec2::new(0.0, 0.0), d_a, '*', styles[4]);
        add_line(&mut grid, Vec2::new(0.0, 0.0), d_b, '@', styles[6]);

        add_line(&mut grid, Vec2::new(0.0, 0.0), face_b, '^', styles[1]);
        add_line(&mut grid, Vec2::new(0.0, 0.0), face_a, '+', styles[7]);
        draw_grid(&mut grid);

        // Assertions to ensure intersection
        assert!(
            num_contacts > 0,
            "Expected contacts, but found {}",
            num_contacts
        );
    }
    #[test]
    fn test_overlap_at_angle_and_fixed() {
        let styles = get_styles();
        let mut grid = make_grid(30);

        // Define overlapping boxes at an angle
        let pos_a = Vec2::new(14.0, 2.0);
        let pos_b = Vec2::new(18.0, 2.0);
        let mut box_a = Body::new(Vec2::new(4.0, 4.0), 1.0);
        box_a.position = pos_a;
        box_a.rotation = 45.0_f32.to_radians();

        let mut box_b = Body::new(Vec2::new(4.0, 4.0), 1.0);
        box_b.position = pos_b;

        // Draw the boxes
        add_box(
            &mut grid,
            pos_a,
            box_a.width,
            box_a.rotation,
            'A',
            styles[4],
        );
        add_box(
            &mut grid,
            pos_b,
            box_b.width,
            box_b.rotation,
            'B',
            styles[6],
        );
        let rot_a = Mat2x2::new_from_angle(box_a.rotation);
        let rot_b = Mat2x2::new_from_angle(box_b.rotation);

        let rot_a_t = rot_a.transpose();
        let rot_b_t = rot_b.transpose();

        let d_p = pos_b - pos_a;
        let d_a = rot_a_t * d_p;
        let d_b = rot_b_t * d_p;

        let c = rot_a_t * rot_b;
        let abs_c = c.abs();
        let abs_c_t = c.abs().transpose();

        let face_a = d_a.abs() - (box_a.width * 0.5) - abs_c * (box_b.width * 0.5);
        let face_b = d_b.abs() - (box_b.width * 0.5) - abs_c_t * (box_a.width * 0.5);
        // Perform collision detection
        let mut contacts = Vec::new();
        let num_contacts = collide(&mut contacts, &box_a, &box_b);
        println!("{:?}", contacts);
        draw_collision_result(&mut grid, &contacts);
        // Draw the grid
        add_line(&mut grid, Vec2::new(0.0, 0.0), d_p, '.', styles[7]);
        add_line(&mut grid, Vec2::new(0.0, 0.0), d_a, '*', styles[4]);
        add_line(&mut grid, Vec2::new(0.0, 0.0), d_b, '@', styles[6]);

        add_line(&mut grid, Vec2::new(0.0, 0.0), face_b, '^', styles[1]);
        add_line(&mut grid, Vec2::new(0.0, 0.0), face_a, '+', styles[7]);

        let normal = if d_a.x > 0.0 { rot_a.col1 } else { -rot_a.col1 };
        println!("The normal Vector is {}", normal);
        println!("The rotation Matrix is {}", rot_a);
        println!("\x1b[2J");

        add_line(&mut grid, Vec2::new(0.0, 0.0), normal, '/', styles[7]);

        draw_grid(&mut grid);

        // Assertions to ensure intersection
        assert!(
            num_contacts > 0,
            "Expected contacts, but found {}",
            num_contacts
        );
    }
}
