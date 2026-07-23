use crate::math_utils::{Mat2x2, Vec2};
use std::sync::atomic::{AtomicUsize, Ordering};

pub struct ConvexPolygon {
    vertices: Vec<Vec2>,
}

impl ConvexPolygon {
    /// Returns a new polygon
    pub fn new(vertices: Vec<Vec2>) -> Self {
        Self { vertices }
    }
    /// Returns the number of vertices in the polygon.
    pub fn get_num_vertices(&self) -> usize {
        self.vertices.len()
    }

    /// Returns the vertex at the given index, handling wraparound.
    pub fn get_vertex(&self, i: isize) -> Vec2 {
        let n = self.get_num_vertices();
        //let index = ((i % n as isize) + n as isize) as usize % n;
        let index: usize = ((i + n as isize + 1) % n as isize) as usize;
        self.vertices[index]
    }

    /// Returns the edge vector from vertex `i` to vertex `(i + 1)`, handling wraparound.
    pub fn get_edge(&self, i: isize) -> Vec2 {
        let v1 = self.get_vertex(i);
        let v2 = self.get_vertex(i + 1);
        v2 - v1
    }

    /// Returns the normal vector of edge `i`, ensuring it points outward.
    pub fn get_normal(&self, i: isize) -> Vec2 {
        let edge = self.get_edge(i);

        Vec2 {
            x: edge.y,
            y: -edge.x,
        }
    }

    /// Calculates the signed area of the polygon.
    pub fn area(&self) -> f32 {
        let n = self.get_num_vertices();
        let mut area = 0.0;
        for i in 0..n {
            let p1 = self.get_vertex(i as isize);
            let p2 = self.get_vertex((i + 1) as isize);
            area += p1.x * p2.y - p1.y * p2.x;
        }
        area.abs() / 2.0
    }
    // Orient the vertices counterclockwise
    fn orient_counterclockwise(&mut self) {
        if self.area() < 0.0 {
            self.vertices.reverse(); // Reverse the vertex order if the area is negative (clockwise)
        }
    }
    /// Calculates the centroid of the polygon.
    pub fn centroid(&self) -> Vec2 {
        let n = self.get_num_vertices();
        let mut cx = 0.0;
        let mut cy = 0.0;
        let mut area = 0.0;

        for i in 0..n {
            let p1 = self.get_vertex(i as isize);
            let p2 = self.get_vertex((i + 1) as isize);
            let cross = p1.x * p2.y - p1.y * p2.x;
            area += cross;
            cx += (p1.x + p2.x) * cross;
            cy += (p1.y + p2.y) * cross;
        }

        area /= 2.0;
        cx /= 6.0 * area;
        cy /= 6.0 * area;

        Vec2 { x: cx, y: cy }
    }

    // Scale the polygon with a factor
    pub fn scale(&mut self, factor: f32) {
        let centroid = self.centroid();

        // Translate to origin, scale, then translate back
        self.vertices = self
            .vertices
            .iter()
            .map(|v| ((*v - centroid) * factor) + centroid)
            .collect();
    }
    /// Calculates the moment of inertia about the centroid.
    pub fn moi(&self) -> f32 {
        let n = self.get_num_vertices();
        let centroid = self.centroid();
        let mut moi = 0.0;

        for i in 0..n {
            let p1 = Vec2 {
                x: self.get_vertex(i as isize).x - centroid.x,
                y: self.get_vertex(i as isize).y - centroid.y,
            };
            let p2 = Vec2 {
                x: self.get_vertex((i + 1) as isize).x - centroid.x,
                y: self.get_vertex((i + 1) as isize).y - centroid.y,
            };

            let cross = p1.x * p2.y - p1.y * p2.x;
            moi += cross
                * (p1.x * p1.x
                    + p1.x * p2.x
                    + p2.x * p2.x
                    + p1.y * p1.y
                    + p1.y * p2.y
                    + p2.y * p2.y);
        }

        moi.abs() / 12.0
    }

    /// Returns the bounding box width and height of the polygon.
    pub fn bounding_box(&self) -> Vec2 {
        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        let mut min_y = f32::MAX;
        let mut max_y = f32::MIN;

        for vertex in &self.vertices {
            if vertex.x < min_x {
                min_x = vertex.x;
            }
            if vertex.x > max_x {
                max_x = vertex.x;
            }
            if vertex.y < min_y {
                min_y = vertex.y;
            }
            if vertex.y > max_y {
                max_y = vertex.y;
            }
        }

        Vec2::new(max_x - min_x, max_y - min_y)
    }

    pub fn rotate(&self, angle: f32) -> ConvexPolygon {
        let center = self.centroid();
        let rotation_mat = Mat2x2::new_from_angle(angle);
        ConvexPolygon {
            vertices: self
                .vertices
                .iter()
                .map(|&vertex| rotation_mat * Vec2::new(vertex.x - center.x, vertex.y - center.y))
                .collect(),
        }
    }

    pub fn translate(&self, position: Vec2) -> ConvexPolygon {
        ConvexPolygon {
            vertices: self
                .vertices
                .iter()
                .map(|&vertex| vertex + position)
                .collect(),
        }
    }

    pub fn get_vertices(&self) -> Vec<Vec2> {
        self.vertices.clone()
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "log", derive(serde::Serialize))]
pub enum Shape {
    #[default]
    Box,
    ConvexPolygon,
    Circle,
}

#[derive(Debug, Default, Clone)]
#[cfg_attr(feature = "log", derive(serde::Serialize))]
pub struct Body {
    pub id: usize,
    pub position: Vec2,
    pub rotation: f32,
    pub velocity: Vec2,
    pub angular_velocity: f32,
    pub force: Vec2,
    pub torque: f32,
    pub width: Vec2,
    pub friction: f32,
    pub mass: f32,
    pub inv_mass: f32,
    pub moi: f32,
    pub inv_moi: f32,
    vertices: Vec<Vec2>,
    pub shape: Shape,
    pub radius: f32,
}

static BODY_ID_COUNTER: AtomicUsize = AtomicUsize::new(1);

impl Body {
    pub fn new(width: Vec2, mass: f32) -> Self {
        let inv_mass;
        let inv_moi;
        let moi;
        if mass < f32::MAX {
            inv_mass = 1.0 / mass;
            moi = mass * (width.x * width.x + width.y * width.y) / 12.0;
            inv_moi = 1.0 / moi;
        } else {
            inv_mass = 0.0;
            moi = f32::MAX;
            inv_moi = 0.0;
        }
        let hw = width.x / 2.0;
        let hh = width.y / 2.0;
        let vertices = vec![
            Vec2 { x: hw, y: hh },
            Vec2 { x: -hw, y: hh },
            Vec2 { x: -hw, y: -hh },
            Vec2 { x: hw, y: -hh },
        ];

        let id = BODY_ID_COUNTER.fetch_add(1, Ordering::Relaxed);

        Self {
            id,
            position: Vec2::new(0.0, 0.0),
            rotation: 0.0,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            friction: 0.0,
            width,
            mass,
            inv_mass,
            inv_moi,
            moi,
            vertices,
            shape: Shape::Box,
            radius: 0.0,
        }
    }
    pub fn new_polygon(vertices: Vec<Vec2>, mass: f32) -> Self {
        let mut convex_polygon = ConvexPolygon {
            vertices: vertices.clone(),
        };
        convex_polygon.orient_counterclockwise();
        let inv_mass;
        let inv_moi;
        let moi;
        if mass < f32::MAX {
            inv_mass = 1.0 / mass;
            moi = mass * convex_polygon.moi();
            inv_moi = 1.0 / moi;
        } else {
            inv_mass = 0.0;
            moi = f32::MAX;
            inv_moi = 0.0;
        }
        let width = convex_polygon.bounding_box();

        let id = BODY_ID_COUNTER.fetch_add(1, Ordering::Relaxed);

        Self {
            id,
            position: Vec2::new(0.0, 0.0),
            rotation: 0.0,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            friction: 0.0,
            width,
            mass,
            inv_mass,
            inv_moi,
            moi,
            vertices,
            shape: Shape::ConvexPolygon,
            radius: 0.0,
        }
    }

    pub fn new_circle(radius: f32, mass: f32) -> Self {
        let inv_mass;
        let inv_moi;
        let moi;
        if mass < f32::MAX {
            inv_mass = 1.0 / mass;
            moi = 0.5 * mass * radius * radius;
            inv_moi = 1.0 / moi;
        } else {
            inv_mass = 0.0;
            moi = f32::MAX;
            inv_moi = 0.0;
        }
        let width = Vec2::new(radius * 2.0, radius * 2.0);
        let vertices = Vec::new();

        let id = BODY_ID_COUNTER.fetch_add(1, Ordering::Relaxed);

        Self {
            id,
            position: Vec2::new(0.0, 0.0),
            rotation: 0.0,
            velocity: Vec2::new(0.0, 0.0),
            angular_velocity: 0.0,
            force: Vec2::new(0.0, 0.0),
            torque: 0.0,
            friction: 0.0,
            width,
            mass,
            inv_mass,
            inv_moi,
            moi,
            vertices,
            shape: Shape::Circle,
            radius,
        }
    }

    pub fn add_force(&mut self, force: Vec2) {
        self.force = self.force + force;
    }

    pub fn get_polygon(&self) -> ConvexPolygon {
        ConvexPolygon {
            vertices: self.vertices.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_body() {
        let body = Body::default();
        println!("{:?}", body);

        let body = Body::new(Vec2::new(1.0, 5.0), 50.0);
        println!("{:?}", body);
    }
    #[test]
    fn test_add_force() {
        let mut body = Body::default();
        body.add_force(Vec2::new(2.0, 5.3));
        assert_eq!(body.force, Vec2::new(2.0, 5.3));
    }
}
