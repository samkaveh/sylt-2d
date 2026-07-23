use std::collections::HashSet;
use std::fmt::{self, Display};
use std::hash::{Hash, Hasher};
use std::ops::{Add, Mul, Neg, Sub};

#[derive(Debug)]
pub enum MathErrors {
    NoInverse { matrix: Mat2x2 },
}

impl Display for MathErrors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MathErrors::NoInverse { matrix } => {
                write!(f, "No inverse could be found for matrix {}", matrix)
            }
        }
    }
}

impl std::error::Error for MathErrors {}

#[derive(Debug, Default, PartialOrd, Clone, Copy)]
#[cfg_attr(feature = "log", derive(serde::Serialize))]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Hash for Vec2 {
    fn hash<H: Hasher>(&self, state: &mut H) {
        // Hash truncated values to avoid issues with floating-point precision
        (self.x.to_bits() >> 4).hash(state);
        (self.y.to_bits() >> 4).hash(state);
    }
}
impl PartialEq for Vec2 {
    fn eq(&self, other: &Self) -> bool {
        (self.x - other.x).abs() < f32::EPSILON && (self.y - other.y).abs() < f32::EPSILON
    }
}

impl Eq for Vec2 {}

pub fn remove_duplicates(vec: Vec<Vec2>) -> Vec<Vec2> {
    let mut seen = HashSet::new();
    vec.into_iter().filter(|item| seen.insert(*item)).collect()
}
impl Vec2 {
    pub fn new(x: f32, y: f32) -> Vec2 {
        Self { x, y }
    }

    pub fn length(self) -> f32 {
        let length = self.x * self.x + self.y * self.y;
        length.sqrt()
    }

    pub fn dot(&self, rhs: Vec2) -> f32 {
        self.x * rhs.x + self.y * rhs.y
    }

    pub fn abs(&self) -> Self {
        Self {
            x: self.x.abs(),
            y: self.y.abs(),
        }
    }
}

impl Display for Vec2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

pub trait Cross<T> {
    type Output;
    fn cross(&self, rhs: T) -> Self::Output;
}

impl Cross<Vec2> for Vec2 {
    type Output = f32;
    fn cross(&self, rhs: Vec2) -> Self::Output {
        self.x * rhs.y - self.y * rhs.x
    }
}

impl Cross<f32> for Vec2 {
    type Output = Vec2;
    fn cross(&self, rhs: f32) -> Self::Output {
        Self {
            x: self.y * rhs,
            y: self.x * -rhs,
        }
    }
}

impl Cross<Vec2> for f32 {
    type Output = Vec2;
    fn cross(&self, rhs: Vec2) -> Self::Output {
        Vec2 {
            x: -self * rhs.y,
            y: self * rhs.x,
        }
    }
}

impl Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl Sub for Vec2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Neg for Vec2 {
    type Output = Vec2;

    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl From<Vec2> for (f32, f32) {
    fn from(v: Vec2) -> Self {
        (v.x, v.y)
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Mat2x2 {
    pub col1: Vec2,
    pub col2: Vec2,
}

impl Mat2x2 {
    pub fn new_from_angle(angle: f32) -> Self {
        let c = f32::cos(angle);
        let s = f32::sin(angle);

        Self {
            col1: Vec2::new(c, s),
            col2: Vec2::new(-s, c),
        }
    }

    pub fn new(col1: Vec2, col2: Vec2) -> Self {
        Self { col1, col2 }
    }

    pub fn transpose(&self) -> Self {
        Self {
            col1: Vec2::new(self.col1.x, self.col2.x),
            col2: Vec2::new(self.col1.y, self.col2.y),
        }
    }

    pub fn invert(&self) -> Result<Self, MathErrors> {
        let a = self.col1.x;
        let b = self.col2.x;
        let c = self.col1.y;
        let d = self.col2.y;
        let det = a * d - b * c;
        if det == 0.0 {
            Err(MathErrors::NoInverse { matrix: *self })
        } else {
            Ok(Self {
                col1: Vec2::new(det * d, -det * c),
                col2: Vec2::new(-det * b, det * a),
            })
        }
    }

    pub fn abs(&self) -> Self {
        Self {
            col1: self.col1.abs(),
            col2: self.col2.abs(),
        }
    }
}

impl Display for Mat2x2 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "[{}, {}\n {}, {}]\n",
            self.col1.x, self.col2.x, self.col1.y, self.col2.y
        )
    }
}

impl Add for Mat2x2 {
    type Output = Mat2x2;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            col1: self.col1 + rhs.col1,
            col2: self.col2 + rhs.col2,
        }
    }
}

impl Mul<Vec2> for Mat2x2 {
    type Output = Vec2;
    fn mul(self, rhs: Vec2) -> Self::Output {
        Vec2 {
            x: self.col1.x * rhs.x + self.col2.x * rhs.y,
            y: self.col1.y * rhs.x + self.col2.y * rhs.y,
        }
    }
}

impl Mul for Mat2x2 {
    type Output = Mat2x2;
    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            col1: self * rhs.col1,
            col2: self * rhs.col2,
        }
    }
}

#[cfg(test)]
mod tests {
    use core::f32;
    use std::f32::consts::PI;

    use super::*;
    #[test]
    fn add_sub_ops() {
        let pos1 = Vec2::new(0.0, 1.0);
        let pos2 = Vec2::new(3.2, 2.9);
        assert_eq!(pos1 + pos2, Vec2::new(3.2, 3.9));
        assert_eq!(pos1 - pos2, Vec2::new(-3.2, -1.9000001));
        assert_eq!(pos1 * 2.0, Vec2::new(0.0, 2.0));
    }

    #[test]
    fn test_cross() {
        let s = 2.0;
        let pos = Vec2::new(1.0, 3.0);
        let res = pos.cross(Vec2::new(3.0, 4.0));
        assert_eq!(res, -5.0);
        let res = s.cross(Vec2::new(3.0, 4.0));
        assert_eq!(res, Vec2::new(-8.0, 6.0));
        let res = pos.cross(s);
        assert_eq!(res, Vec2::new(6.0, -2.0));
        let res = pos.dot(Vec2::new(3.0, 4.0));
        assert_eq!(res, 15.0);
    }

    #[test]
    fn test_length() {
        let pos1 = Vec2::new(1.0, 1.0);
        assert_eq!(pos1.length(), f32::sqrt(2.0));
    }

    #[test]
    fn test_mat() {
        let mat1 = Mat2x2::new_from_angle(PI / 2.0);
        //println!("{}", mat1);
        //println!("{}", mat1.invert());
        //println!("{}", mat1.transpose());
        assert_eq!(mat1.invert().unwrap().col2.x, 1.0);
        assert_eq!(mat1.transpose().col1.y, -1.0);
    }

    #[test]
    fn test_mat_ops() {
        let mat1 = Mat2x2::new_from_angle(PI / 4.0);
        let mat2 = Mat2x2::new_from_angle(-PI / 4.0);
        let pos = Vec2::new(2.0, 3.0);
        //println!("{}", mat1);
        //println!("{}", mat2);
        let res = mat1 * mat2;
        assert_eq!(res.col2.x, 0.0);
        //println!("{}", res);
        let res = mat1 + mat2;
        assert_eq!(res.col1.x, f32::consts::SQRT_2);
        //println!("{}", res);
        let res = mat1 * pos;
        assert!(res.x - f32::consts::FRAC_1_SQRT_2 < f32::EPSILON);
        //println!("{} * {} = {}", mat1, pos, res);
    }
}
